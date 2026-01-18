.PHONY: build run test clean docker-build docker-run

build:
	cd rust_engine && maturin develop

run:
	export PYTHONPATH=$${PYTHONPATH}:src && python src/main.py

test:
	pytest tests/

clean:
	rm -rf rust_engine/target
	find . -type d -name "__pycache__" -exec rm -rf {} +

docker-build:
	docker-compose build

docker-run:
	docker-compose up
