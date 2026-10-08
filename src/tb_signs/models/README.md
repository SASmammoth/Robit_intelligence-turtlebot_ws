# 모델 파일 위치

이 폴더에 `best.engine`을 두면 빌드할 때 함께 설치됩니다.
엔진은 반드시 **이 Jetson에서** 만들어야 합니다 (다른 PC/버전에서 만든 엔진은 안 열림).

    /usr/src/tensorrt/bin/trtexec --onnx=best.onnx --saveEngine=best.engine --fp16
