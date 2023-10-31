./launch_controls.sh
sleep 5
cd PublicApi/
gunicorn -c gunicorn_config.py main:app --access-logfile access.log --error-logfile error.log
