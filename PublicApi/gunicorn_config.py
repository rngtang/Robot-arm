import os



workers = int(os.environ.get('GUNICORN_PROCESSES', '1'))

# threads = int(os.environ.get('GUNICORN_THREADS', '4'))

# timeout = int(os.environ.get('GUNICORN_TIMEOUT', '120'))

# bind = os.environ.get('GUNICORN_BIND', '10.194.72.227:5000')

# new IP address: 
# bind = os.environ.get('GUNICORN_BIND', '10.194.29.175:5000')
bind = os.environ.get('GUNICORN_BIND', '0.0.0.0:5000')


forwarded_allow_ips = '*'

# secure_scheme_headers = { 'X-Forwarded-Proto': 'https' }

