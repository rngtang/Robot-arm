./launch_controls.sh

cd PublicApi/
gunicorn -b 10.194.72.227:5000 main.wsgi:app
