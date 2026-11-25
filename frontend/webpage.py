import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent

PREDICTOR_DIR = (
    PROJECT_ROOT
    / "software"
    / "Firmware_Targets"
    / "Jetson"
    / "software_predictor"
)

sys.path.insert(0, str(PREDICTOR_DIR))

from flask import (
    Flask,
    Response,
    render_template,
    jsonify,
    redirect
)
import logging


log = logging.getLogger("werkzeug")
log.setLevel(logging.ERROR)
PROJECT_ROOT = Path(__file__).resolve().parent.parent


sys.path.append(str(PREDICTOR_DIR))

import BANG
import screencapGrabber

from screencapGrabber import generate_frames, trackingStatus
from BANG import get_coords
from flask import request

app = Flask(__name__)
@app.route("/")
def index():
    return render_template("dashboard.html")


@app.route("/fire", methods=["GET", "POST"])
def fire():

    return render_template("camera.html")


#<!-- Manual control. Click event to denote where the click occured. we'll pass it to get_coords through a python event and it should follow the normal route after -->
@app.route("/manual_click", methods=["POST"])
def manual_click():
    data = request.get_json()

    click_x = data["x"]
    click_y = data["y"]
    displayed_width = data["width"]
    displayed_height = data["height"]

    xyz = screencapGrabber.get_manual_xyz(
        click_x,
        click_y,
        displayed_width,
        displayed_height
    )

    if xyz is None:
        print("[MANUAL] No valid depth at clicked point")
        return jsonify({"ok": False})

    x, y, z = xyz

    yawDegrees, pitchDegrees = BANG.get_coords(x, y, z)

    print(
        f"[MANUAL] XYZ=({x:.3f}, {y:.3f}, {z:.3f}) "
        f"yaw={yawDegrees:.2f} pitch={pitchDegrees:.2f}"
    )


    # send yawDegrees / pitchDegrees to gimbal I2C here

    return jsonify({
        "ok": True,
        "x": x,
        "y": y,
        "z": z,
        "yaw": yawDegrees,
        "pitch": pitchDegrees
    })

@app.route("/video_feed")
def video_feed():

    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )

@app.route("/status")
def status():
    return jsonify(trackingStatus)

@app.route("/reset")
def reset():
    BANG.reset()
    return redirect("/")


if __name__ == "__main__":

    app.run(
        host="0.0.0.0",
        port=5001
    )