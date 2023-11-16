# ./launch_controls.sh
# ^^ commented out because it potentially takes some time for ./launch_controls.py to execute, just run ./launch_controls.sh manually

# first kill all past tasks that might have not ended correctly
sudo fuser -k 5000/tcp
kill -9 `jobs -ps`

cd PublicApi/
# nohup gunicorn -c gunicorn_config.py main:app --log-level debug &
python3 main.py &
sleep 5
cd ..
./launch_controls.sh