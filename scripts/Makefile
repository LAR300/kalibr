# Makefile for Kalibr Docker operations
# Target camera: ZED Camera

IMAGE_NAME = kalibr
CONTAINER_NAME = kalibr_zed
DOCKERFILE = Dockerfile_ros1_20_04
HOST_DATA_DIR = $(shell pwd)/data
CONTAINER_DATA_DIR = /data

.PHONY: help build run start stop exec clean restart status

help:
	@echo "Available commands:"
	@echo "  make build   - Build the Docker image using $(DOCKERFILE)"
	@echo "  make run     - Create and run a new container with GUI (X11) support and volume sharing"
	@echo "  make start   - Start the existing stopped container"
	@echo "  make stop    - Stop the running container"
	@echo "  make restart - Restart the container"
	@echo "  make exec    - Open a new bash terminal in the running container"
	@echo "  make status  - Show status of the container"
	@echo "  make clean   - Remove the container"

build:
	@echo "Building Docker image $(IMAGE_NAME)..."
	docker build -t $(IMAGE_NAME) -f ../$(DOCKERFILE) ..

run:
	@echo "Ensuring host data directory exists at $(HOST_DATA_DIR)..."
	mkdir -p $(HOST_DATA_DIR)
	@echo "Allowing local connections to X11 display..."
	-xhost +local:root
	@echo "Running new container $(CONTAINER_NAME)..."
	docker run -it \
		--name $(CONTAINER_NAME) \
		--privileged \
		--net=host \
		--ipc=host \
		-e DISPLAY=$(DISPLAY) \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v $(HOST_DATA_DIR):$(CONTAINER_DATA_DIR):rw \
		$(IMAGE_NAME)

start:
	@echo "Starting container $(CONTAINER_NAME)..."
	docker start $(CONTAINER_NAME)
	@echo "Attaching to container $(CONTAINER_NAME)..."
	docker attach $(CONTAINER_NAME)

stop:
	@echo "Stopping container $(CONTAINER_NAME)..."
	docker stop $(CONTAINER_NAME)

restart:
	@echo "Restarting container $(CONTAINER_NAME)..."
	docker restart $(CONTAINER_NAME)

exec:
	@echo "Executing bash in running container $(CONTAINER_NAME)..."
	docker exec -it $(CONTAINER_NAME) /bin/bash

status:
	@echo "Container status:"
	docker ps -a --filter name=$(CONTAINER_NAME)

clean:
	@echo "Removing container $(CONTAINER_NAME)..."
	-docker rm -f $(CONTAINER_NAME)
