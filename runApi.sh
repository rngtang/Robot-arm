# ./launch_controls.sh
# ^^ commented out because it potentially takes some time for ./launch_controls.py to execute, just run ./launch_controls.sh manually

cd PublicApi/
gunicorn -c gunicorn_config.py main:app --log-level debug 
