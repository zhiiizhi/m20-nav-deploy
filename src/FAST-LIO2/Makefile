all: arm64

help:
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@echo "   1. make build              - build all images"
	# @echo "   1. make pull             - pull all images"
	@echo "   1. make clean              - remove all images"
	@echo ""

arm64:
	@docker build --platform=linux/arm64 -t buaadocker.xuhao1.me/fastlio2:arm64 -f ./Dockerfile .  

pc:
	@docker build --platform=linux/amd64 -t buaadocker.xuhao1.me/fastlio2:amd64 -f ./Dockerfile  --build-arg BASE_IMAGE="buaadocker.xuhao1.me/ros_base_noetic:amd64" .  

upload_arm64: arm64
	@docker push buaadocker.xuhao1.me/fastlio2:arm64

upload_amd64: pc
	@docker push buaadocker.xuhao1.me/fastlio2:amd64

upload: upload_amd64 upload_arm64

clean:
	@docker rmi -f buaadocker.xuhao1.me/fastlio2:amd64
	@docker rmi -f buaadocker.xuhao1.me/fastlio2:arm64
