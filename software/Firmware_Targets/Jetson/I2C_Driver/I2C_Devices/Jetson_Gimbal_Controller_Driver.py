#file to get coords and turn them into angles, and send them to I2C controller/Gimbal
from struct import pack, unpack
from ..I2C_Handler import I2CTransaction, Jetson_I2C_Handler, I2CResult
from .address_map_grabber import address_map_grabber


class GimbalControllerDriver:
    def __init__(self, I2C_Handler: Jetson_I2C_Handler, YAW_ADDRESS: int, PITCH_ADDRESS: int, register_map: dict[str, int] | None = None):
        self._I2C_Handler = I2C_Handler
        self._register_map = register_map if register_map is not None else address_map_grabber("../../../Gimbal_Controller/config.h")
        I2C_Handler.register_device("Yaw Controller", YAW_ADDRESS, False)
        I2C_Handler.register_device("Pitch Controller", PITCH_ADDRESS, False)
        
        self._Yaw_I2C_Address = YAW_ADDRESS
        self._Pitch_I2C_Address = PITCH_ADDRESS
        
    def YAW_Ready(self) -> I2CResult[bool]:
        """Checks if the YAW controller is ready
        
        Returns:
            I2CResult[bool]: True if the YAW controller is ready
        """
        
        def decode_ready_bit(data: bytes) -> bool:
            """Decodes the ready bit"""
            return data[0] == 1
        
        transaction = I2CTransaction(
            deviceAddress=self._Yaw_I2C_Address,
            registerAddress=self._register_map["READY"],
            readCount=1
        )
        
        self._I2C_Handler.submitTransaction(transaction)
        return I2CResult(transaction, decode_ready_bit)
    
    def PITCH_Ready(self) -> I2CResult[bool]:
        """Checks if the PITCH controller is ready
        
        Returns:
            I2CResult[bool]: True if the PITCH controller is ready
        """
        
        def decode_ready_bit(data: bytes) -> bool:
            """Decodes the ready bit"""
            return data[0] == 1
        
        transaction = I2CTransaction(
            deviceAddress=self._Pitch_I2C_Address,
            registerAddress=self._register_map["READY"],
            readCount=1
        )
        
        self._I2C_Handler.submitTransaction(transaction)
        return I2CResult(transaction, decode_ready_bit)
    
    def set_target_angles(self, yaw: float, pitch: float) -> tuple[I2CResult[None], I2CResult[None]]:
        """Sets the target angles of the pitch and yaw axis
        
        Returns: a tuple containting (yaw result, pitch result)
        """
        
        pitch_transaction = I2CTransaction(
            deviceAddress=self._Pitch_I2C_Address,
            registerAddress=self._register_map["TARGET_ANGLE"],
            writeData=pack("<f", pitch),
            readCount=0,
        )
        
        yaw_transaction = I2CTransaction(
            deviceAddress=self._Yaw_I2C_Address,
            registerAddress=self._register_map["TARGET_ANGLE"],
            writeData=pack("<f", yaw),
            readCount=0,
        )
        
        self._I2C_Handler.submitTransaction(pitch_transaction)
        self._I2C_Handler.submitTransaction(yaw_transaction)
        
        return (I2CResult(yaw_transaction), I2CResult(pitch_transaction))

    def read_target_pitch(self) -> I2CResult[float]:
        """Reads the target angle of pitch
        
        Returns:
            I2CResult[float]: The target pitch
        """


        def decode_data(data: bytes) -> float:
            """Decodes the data from controller's response"""
            return unpack("<f", data)[0]

        pitch_transaction = I2CTransaction(
            deviceAddress=self._Pitch_I2C_Address,
            registerAddress=self._register_map["TARGET_ANGLE"],
            readCount=4
        )

        self._I2C_Handler.submitTransaction(pitch_transaction)
        return I2CResult(pitch_transaction, decode_data)

    def read_target_yaw(self) -> I2CResult[float]:
            """Reads the target angle of yaw
            
            Returns:
                I2CResult[float]: The target yaw
            """
    
    
            def decode_data(data: bytes) -> float:
                """Decodes the data from controller's response"""
                return unpack("<f", data)[0]
    
            pitch_transaction = I2CTransaction(
                deviceAddress=self._Yaw_I2C_Address,
                registerAddress=self._register_map["TARGET_ANGLE"],
                readCount=4
            )
    
            self._I2C_Handler.submitTransaction(pitch_transaction)
            return I2CResult(pitch_transaction, decode_data)

    def read_current_yaw(self) -> I2CResult[float]:
        """Read the measured yaw in radians, rather than the target."""
        return self._read_current(self._Yaw_I2C_Address)

    def read_current_pitch(self) -> I2CResult[float]:
        """Read the measured pitch in radians, rather than the target."""
        return self._read_current(self._Pitch_I2C_Address)

    def _read_current(self, address: int) -> I2CResult[float]:
        transaction = I2CTransaction(
            deviceAddress=address,
            registerAddress=self._register_map["CURRENT_ANGLE"],
            readCount=4,
        )
        self._I2C_Handler.submitTransaction(transaction)
        return I2CResult(transaction, lambda data: unpack("<f", data)[0])
