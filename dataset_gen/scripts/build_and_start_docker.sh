#!/bin/bash

set -e

docker build -t dataset_gen_image .

./scripts/start_docker.sh
