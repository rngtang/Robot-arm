# for the robot
from flask import Flask, request

app = Flask(__name__)
# if __name__ == '__main__':
#     app.run(host='0.0.0.0',port=5000,debug=True)

@app.route("/")
def hello_world():
    return "<p>Hello, World!</p>"

# position is two numbers (row, col) put together
@app.route("/move")
def access_position():
    position = request.args.get('pos')
    result = robot_move(position)
    return result

def robot_move(position):
    return "hi zhichen"
    # return '''<h1>The given position is: {}, {}</h1>'''.format(row, col)
