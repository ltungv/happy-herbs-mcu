.PHONY: build clean deploy gomodgen

build: gomodgen
	export GO111MODULE=on
	 # FIX: Add CGO_ENABLED=0 when building go files
	 # Github issue: https://github.com/serverless/serverless/issues/8539
	env CGO_ENABLED=0 GOOS=linux go build -ldflags="-s -w" -o bin/hello hello/main.go
	env CGO_ENABLED=0 GOOS=linux go build -ldflags="-s -w" -o bin/world world/main.go

clean:
	rm -rf ./bin ./vendor go.sum

deploy: clean build
	sls deploy --verbose

gomodgen:
	chmod u+x gomod.sh
	./gomod.sh
