FROM mcr.microsoft.com/devcontainers/universal:2

RUN cd /opt && wget 'https://github.com/wpilibsuite/vscode-wpilib/releases/download/v2023.2.1/vscode-wpilib-2023.2.1.vsix' && code install --install-extension ./vscode-wpilib-2023.2.1.vsix && apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install git

