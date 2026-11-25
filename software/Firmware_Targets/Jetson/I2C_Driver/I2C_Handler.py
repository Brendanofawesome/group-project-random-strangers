from __future__ import annotations
import threading
from dataclasses import dataclass, field
from queue import Empty, Queue
from time import time, monotonic
from typing import Callable, ClassVar, Generic, TypeVar
import logging

from smbus2 import SMBus, i2c_msg

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

ResultType = TypeVar("ResultType")
class I2CResult(Generic[ResultType]):
    """Describes the result from an I2C Transaction, functions as a future

    Args:
        transaction: The I2CTransaction that was submitted to the driver
        decoder: An optional function that takes the bytes returned by the
            transaction and decodes it into a more useful type. Leave this
            unset for transactions with no result.
    """
    def __init__(
        self,
        transaction: I2CTransaction,
        decoder: Callable[[bytes], ResultType] | None = None,
    ) -> None:
        self._transaction = transaction
        self._decoder = decoder

    def done(self) -> bool:
        return self._transaction.done.is_set()

    def wait(self, timeout: float | None = None) -> bool:
        return self._transaction.done.wait(timeout)

    def result(self, timeout: float | None = None) -> ResultType | None:
        if not self._transaction.done.wait(timeout):
            raise TimeoutError("I2C transaction did not complete in time")

        if self._transaction.error is not None:
            raise self._transaction.error

        if self._decoder is None:
            return None
        elif self._transaction.readData is None:
            raise RuntimeError("I2C transaction completed without a response")
        else:
            return self._decoder(self._transaction.readData)

@dataclass
class I2CTransaction:
    """Describes one queued I2C write, read, or combined transaction.

    Set ``deviceAddress`` to the 7-bit I2C address.  Set ``registerAddress`` to
    the register address to send before a write/read, or leave it at ``None`` when
    the device does not use a register address. Put outgoing bytes in
    ``writeData`` and set ``readCount`` to the number of bytes to read. For a
    read, the driver appends the bytes received from the bus to that buffer at ''readData''.

    A transaction is submitted by placing it on the driver's queue
    and waiting on ``done``. The worker sets ``done`` after a successful bus
    operation. The worker also uses ``inProgressLock`` while validating and
    executing the transaction, so callers should not modify transaction fields
    concurrently with processing. To reuse a transaction, clear the ''done'' event.
    """

    deviceAddress: int = 0
    registerAddress: int | None = None

    writeData: bytes = bytes(0)

    readCount: int = 0
    readData: bytes | None = field(default=None, init=False)

    inProgressLock: threading.Lock = field(default_factory=threading.Lock, init=False)
    error: Exception | None = field(default=None, init=False)
    done: threading.Event = field(default_factory=threading.Event, init=False)
    

class Jetson_I2C_Handler:
    """Owns one i2c bus and asynchronously fulfuls transactions added to the Queue

    Only one active instance is allowed per I2C bus. The caller owns the
    transaction queue and is responsible for starting ``runThread`` with
    ``start()`` after construction.  Devices must be registered before their
    addresses can be used by ordinary transactions.
    """

    # One driver instance per I2C bus
    _error_check_period_s: ClassVar[float] = 0.25
    _instances: ClassVar[dict[int, Jetson_I2C_Handler]] = {}
    _instance_lock: ClassVar[threading.Lock] = threading.Lock()

    def __init__(self, i2c_bus_id: int = 7, transaction_queue: Queue[I2CTransaction | None] | None = None, doErrorChecking: bool = False) -> None:
        """Initialize the singleton driver for ``i2c_bus_id``.

        Args:
            i2c_bus_id: Linux/smbus bus number.
            transaction_queue: Queue consumed by :meth:`_runner_task`.  Put
                ``I2CTransaction`` objects on it; ``None`` is reserved as the
                worker shutdown sentinel.
            doErrorChecking: When true, the worker periodically polls all
                registered devices that advertise error support.  Those polls
                share the bus with normal transactions

        Raises:
            OSError: If ``smbus2`` cannot open the requested bus.
        """

        # Prevent init from resetting an existing instance.
        with self._instance_lock:
            if getattr(self, "_initialized", False):
                return

            try:
                if(transaction_queue is None):
                    transaction_queue = Queue()

                self._i2c_bus_id = i2c_bus_id
                self._transaction_queue: Queue[I2CTransaction | None] = transaction_queue

                self._devices: dict[int, tuple[str, bool]] = {}
                self.errors: dict[str, list[str]] = {}
                self.dataLock: threading.Lock = threading.Lock()

                self._bus = SMBus(i2c_bus_id)

                self.doErrorChecking: bool = doErrorChecking
                self.lastErrorCheck: float = float("NaN")
                self.runThread: threading.Thread = threading.Thread(target=self._runner_task, daemon=True)

                logger.debug(f"Initialized an I2C Driver on BusID:{i2c_bus_id}")
                self._initialized = True
                self._stopping = False

            except OSError:
                #remove failed instances
                self._instances.pop(i2c_bus_id, None)
                raise
    
    def submitTransaction(self, transaction: I2CTransaction) -> None:
        """Queue a fresh transaction; its I2CResult reports success or failure."""
        with self.dataLock:
            if not self._initialized or self._stopping:
                raise RuntimeError("I2C driver is closed or stopping")
            self._transaction_queue.put(transaction)

    def startDriverThread(self) -> None:
        with self.dataLock:
            if self.runThread.is_alive():
                return
            if self._stopping or self.runThread.ident is not None:
                raise RuntimeError("Create a new driver after stopping it")
            self.runThread.start()

    def stopDriverThread(self) -> None:
        # Never join while holding dataLock: the worker also needs it.
        with self.dataLock:
            if not self._stopping:
                self._stopping = True
                self._transaction_queue.put(None)
        if self.runThread.is_alive():
            self.runThread.join(timeout=2.0)
            if self.runThread.is_alive():
                raise TimeoutError("I2C worker did not stop; bus may be blocked")

    def close(self) -> None:
        if not getattr(self, "_initialized", False):
            return
        self.stopDriverThread()
        with self._instance_lock:
            self._bus.close()
            self._instances.pop(self._i2c_bus_id, None)
            self._initialized = False

    def register_device(self, device_name: str, i2cAddress: int, errors_supported: bool) -> bool:
            """Register a device address for transaction validation.

            Args:
                device_name: Name used as the key in ``errors`` when error
                    polling reports a device error.
                i2cAddress: 7-bit address.  Only ``0x08`` through ``0x77``
                    inclusive are accepted; reserved addresses are rejected.
                errors_supported: If true, periodic error checking queries the
                    device using the driver's standard error protocol.

            Returns:
                ``True`` when the address was newly registered; ``False`` when
                it is outside the accepted range or is already registered.
            """

            if(i2cAddress > 7 and i2cAddress < 120 and i2cAddress not in self._devices): #do not allow reserved addresses or device conflicts
    
                with self.dataLock:
                    self._devices[i2cAddress] = (device_name, errors_supported)
    
                return True
    
            return False

    def update_errors(self) -> None:
            """Poll registered error-reporting devices and return current errors.
            
            
            This method is called automatically by the driver thread if error 
            checking is enabled. This method performs blocking I2C reads/writes.
            """
    
            all_new_errors: dict[str, list[str]] = dict()
            with self.dataLock:
                devices = list(self._devices.items()) #snapshot


            for device_address, (device_name, errors_supported) in devices:
                if(errors_supported == False):
                    continue

                #querry errors
                error_metadata = self._bus.read_i2c_block_data(device_address, 0x1, 5)
                error_integer = int.from_bytes(error_metadata[1:4], byteorder="big", signed=False)
                device_errors: list[str] = list()

                if(error_metadata[0] != 0): #error(s) detected

                    #loop through each set error bit
                    while(error_integer > 0):
                        lowest_set_bit_pos = (error_integer & -error_integer).bit_length()
                        error_integer &= error_integer - 1

                        #command the device to provide the error corresponding to the set bit
                        self._bus.write_byte_data(device_address, 0x6, lowest_set_bit_pos)

                        #get the error string reads a number of bytes = to the first read value
                        error_data = self._bus.read_block_data(device_address, 0x7)

                        device_errors.append("".join(chr(byte) for byte in error_data))

                #add device errors to the error dictionary
                all_new_errors[device_name] = device_errors

            with self.dataLock: #save new errors
                self.lastErrorCheck = time()
                self.errors = all_new_errors

    def getAllErrors(self) -> tuple[float, dict[str, list[str]]]:
        """Return a snapshot of the most recently reported I2C errors.

        Returns:
            A timestamp of when the last errors were read and a dictionary
            whose keys are registered device names and whose values are 
            error messages.
        """

        with self.dataLock:
            return (self.lastErrorCheck, {device_name: errors.copy() for device_name, errors in self.errors.items()})

    def getDeviceErrors(self, deviceName: str) -> tuple[float, list[str]]:
        """Return a snapshot of the most recently reported I2C errors from a specific device.

        Returns:
            A timestamp of when the last errors were read and a list of error messages.
        """
        with self.dataLock:
            device_errors = self.errors.get(deviceName)
            if(device_errors is None):
                logger.warning(f"")
                device_errors = []
            else:
                device_errors = device_errors.copy()

            return (self.lastErrorCheck, device_errors)
        
    def _runner_task(self) -> None:
        """Consume and execute transactions until a ``None`` sentinel arrives.

        Method should be in dedicated thread. Call :meth:`startDriverThread` to 
        do this automatically
        """
        
        next_error_check = monotonic()
        while True:
            if self.doErrorChecking and monotonic() >= next_error_check:
                try:
                    self.update_errors()
                except Exception:
                    logger.exception("Could not poll I2C errors")
                next_error_check = monotonic() + self._error_check_period_s
            timeout = max(0.0, next_error_check - monotonic()) if self.doErrorChecking else None
            try:
                transaction = self._transaction_queue.get(timeout=timeout)
            except Empty:
                continue
            if transaction is None:
                break
            with transaction.inProgressLock:
                try:
                    reason = self._validate_transaction(transaction)
                    if reason:
                        raise ValueError(reason)
                    self._perform_transaction(transaction)
                except Exception as error:
                    transaction.error = error
                finally:
                    # Also finish invalid/failed transactions so callers never hang.
                    transaction.done.set()

    def _validate_transaction(self, transaction: I2CTransaction) -> str | None:
        address = transaction.deviceAddress
        if not isinstance(address, int) or not 0x08 <= address <= 0x77:
            return "Expected a nonreserved 7-bit I2C device address"
        if address not in self._devices:
            return "I2C transaction to an unregistered device"
        register = transaction.registerAddress
        if register is not None and (not isinstance(register, int) or not 0 <= register <= 0xFF):
            return "Expected a one-byte register address"
        if not isinstance(transaction.writeData, bytes):
            return "writeData must be bytes"
        if not isinstance(transaction.readCount, int) or transaction.readCount < 0:
            return "readCount must be a nonnegative integer"
        if transaction.done.is_set():
            return "Submit a new transaction instead of reusing a completed one"
        return None

    def _perform_transaction(self, transaction: I2CTransaction) -> None:
        # The runner owns transaction.inProgressLock. Acquire only the bus lock.
        with self.dataLock:
            payload = transaction.writeData
            if transaction.registerAddress is not None:
                payload = bytes([transaction.registerAddress]) + payload
            messages = []
            if payload:
                messages.append(i2c_msg.write(transaction.deviceAddress, payload))
            read_message = None
            if transaction.readCount:
                read_message = i2c_msg.read(transaction.deviceAddress, transaction.readCount)
                messages.append(read_message)
            if not messages:
                raise ValueError("Empty I2C transaction")
            self._bus.i2c_rdwr(*messages)
            if read_message is not None:
                transaction.readData = bytes(read_message)

    #ensures that there is only one active driver per bus
    def __new__(cls, *args, **kwargs) -> Jetson_I2C_Handler:
        """Return the existing driver for ``i2c_bus_id`` or allocate one.
        """

        with cls._instance_lock:
            instance = cls._instances.get(i2c_bus_id)

            if instance is None:
                instance = super().__new__(cls)
                cls._instances[i2c_bus_id] = instance

            return instance
