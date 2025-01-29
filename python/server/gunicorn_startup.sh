#!/bin/bash
exec gunicorn -b :9988 --access-logfile - --error-logfile - server:app