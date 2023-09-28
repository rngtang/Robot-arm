# for the robot
from flask import Flask, request

app = Flask(__name__)

@app.route("/")
def hello_world():
    return "<p>Hello, World!</p>"

# GET endpoint for robot to get where it should move to
# position is two numbers (row, col) put together
@app.route('/move/<position>', methods=['GET'])
@app.route('/move/', defaults={'position' : '99'})
def move(position):
    response = request.get(url="http://0.0.0.0:5000/send/")
        # row = request.get(position[0])
        # col = request.get(position[1])

    row = position[0]
    col = position[1]
    return '''<h1>The given position is: {}, {}</h1>'''.format(row, col)

if __name__ == '__main__':
    app.run(host='0.0.0.0',port=5000,debug=True)