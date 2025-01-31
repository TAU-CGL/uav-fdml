from flask import Flask, request, jsonify
import threading

app = Flask(__name__)

# Shared data to store the most recent measurement
latest_measurement = None
measurement_lock = threading.Lock()

@app.route('/post_measurement', methods=['POST'])
def post_measurement():
    global latest_measurement

    try:
        data = request.json
        if not data or 'measurement' not in data:
            return jsonify({"error": "Invalid input"}), 400

        with measurement_lock:
            latest_measurement = data['measurement']

        return jsonify({"status": "Measurement received"}), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/get_recent_measurement', methods=['GET'])
def get_recent_measurement():
    global latest_measurement

    try:
        with measurement_lock:
            if latest_measurement is None:
                return jsonify({"error": "No measurement available"}), 404

            return jsonify({"measurement": latest_measurement}), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/is_working', methods=['GET'])
def is_working():
    return "YES :)", 418

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=9988)
