from flask import Flask, request, send_file, jsonify
import subprocess, tempfile, os, shutil

app = Flask(__name__)

@app.route("/build", methods=["POST"])
def build():
    # raw ZIP in request.data
    zip_bytes = request.data
    tmp = tempfile.mkdtemp()
    zip_path = os.path.join(tmp, "project.zip")
    with open(zip_path, "wb") as f:
        f.write(zip_bytes)

    # unzip
    subprocess.run(["unzip", "-q", zip_path, "-d", tmp], check=True)

    # assume there's a Makefile at tmp/Makefile that builds firmware.bin
    # adjust as needed (e.g. MCUXpresso CLI or CMake)
    res = subprocess.run(["make"], cwd=tmp, capture_output=True, text=True)
    if res.returncode != 0:
        # return build errors
        return jsonify({"error": "build failed", "stdout": res.stdout, "stderr": res.stderr}), 400

    bin_path = os.path.join(tmp, "build", "firmware.bin")
    if not os.path.isfile(bin_path):
        return jsonify({"error": "firmware.bin not found"}), 500

    # stream it back
    return send_file(bin_path, mimetype="application/octet-stream")

if __name__ == "__main__":
    # Listen on all interfaces, port 5000
    app.run(host="0.0.0.0", port=5000)
