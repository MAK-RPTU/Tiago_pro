#!/bin/bash

xhost +local:root
docker start 1ecead2ed6a9
docker exec -it tiago_pro bash
