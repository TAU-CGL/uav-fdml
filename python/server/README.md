# FDML Server

This is a simple Flask server, that allows for two remote computers communicate via the cloud. One passes a single message (which is a collection of measurements). The other - reads that message.

## Setup

The following setup was done on an Ubuntu Server 22.04 LTS (without docker).
You may use any other setup you'd like.

We serve the Flask server with gunicorn & nginx.

1. Copy the `python/server` folder to your cloud computer.
2. Make sure the `gunicorn_startup.sh` is executable: 
    ```
    chmod a+x gunicorn_startup.sh
    ```
2. Modify the content of the `fdml.service` file; change `<YOUR_USERNAME>` and `<SERVER_ROOT_DIR>` accorindgly.
Copy the file to `/etc/systemd/system`. Enable the service:
    ```
    sudo systemctl start fdml.service 
    ```
3. Modify the content of the `fdml_nginx` file; change `<YOUR_SERVER_NAME>` to the hostname (ip address or DNS record). Copy the file to `/etc/nginx/sites-available`, and add a symlink:
    ```
    sudo ln -s /etc/nginx/sites-available/fdml_nginx /etc/nginx/sites-enabled/fdml_nginx
    ```
4. Test that the server is up and running:
    ```
    http://<YOUR_SERVER_NAME>:9989/is_working
    ```