./launch_controls.sh

cd PublicApi/
gunicorn -c gunicorn_config.py main:app
