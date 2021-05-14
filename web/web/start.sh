#!/bin/bash
while ! nc -z rabbit 5672; do sleep 3; done
pm2-runtime server.js