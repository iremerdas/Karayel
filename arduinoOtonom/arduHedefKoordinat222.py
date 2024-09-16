from flask import Flask, request, jsonify

app = Flask(__name__)


@app.route('/set_coordinates', methods=['POST'])
def set_coordinates():
    data = request.get_json()
    lat = data['latitude']
    lon = data['longitude']
    print(f"Gelen koordinatlar: Lat={lat}, Lon={lon}")
    # Bu koordinatları aracın hedefine aktarmak için burada işle
    return jsonify({'status': 'success', 'latitude': lat, 'longitude': lon})


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
