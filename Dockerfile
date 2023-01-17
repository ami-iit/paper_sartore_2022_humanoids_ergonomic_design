# syntax=docker/dockerfile:1

ARG VARIANT="focal"
FROM mcr.microsoft.com/vscode/devcontainers/base:0-${VARIANT}
ARG USERNAME=user
ARG USERID=1001
ARG GIT_USER=$USERNAME
ARG GIT_USEREMAIL=$USERNAME@$USERNAME.$USERNAME
SHELL ["/bin/bash", "-c"]

# Set the locale
RUN apt install -y -qq apt-utils locales && rm -rf /var/lib/apt/lists/*
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Install essentials
RUN apt-get update 
COPY deps.sh .
RUN  chmod +x ./deps.sh
RUN ./deps.sh && rm ./deps.sh && rm -rf /var/lib/apt/lists/

RUN pip install numpy==1.22

# #Get coinbrew
COPY coinhsl.zip . 
RUN mkdir CoinIpopt 
RUN cd CoinIpopt  && wget https://raw.githubusercontent.com/coin-or/coinbrew/1cd6e53a9c6f3b235939da3ebf6cabaa14da44bd/coinbrew 
RUN cd CoinIpopt  && chmod u+x coinbrew 
RUN cd CoinIpopt  && ./coinbrew build Ipopt --main-proj-version=releases/3.13.4 --prefix=install --test --no-prompt --verbosity=3 
RUN cd CoinIpopt  && unzip /coinhsl.zip 
RUN cd CoinIpopt  && mv coinhsl-2019.05.21 /CoinIpopt/ThirdParty/HSL/coinhsl 
RUN cd CoinIpopt && ./coinbrew build Ipopt --main-proj-version=releases/3.13.4 --prefix=install --test --no-prompt --verbosity=3 
RUN cd CoinIpopt && ./coinbrew install Ipopt --no-prompt 
RUN cd CoinIpopt/install/lib &&ln -s libcoinhsl.so libhsl.so 
RUN cd CoinIpopt && rm -rf build 

RUN echo 'export IPOPT_DIR=/CoinIpopt/install' >> ~/.bashrc && \
    echo 'export PKG_CONFIG_PATH=${PKG_CONFIG_PATH}:${IPOPT_DIR}/lib/pkgconfig' >> ~/.bashrc && \
    echo 'export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${IPOPT_DIR}/lib' >> ~/.bashrc && \
    echo 'export PATH=${PATH}:${IPOPT_DIR}/lib' >> ~/.bashrc

RUN git config --global user.name ${GIT_USER} && \
    git config --global user.email ${GIT_USEREMAIL} 
RUN git config --global advice.detachedHead false
ENV IPOPT_DIR=/CoinIpopt/install
ENV PKG_CONFIG_PATH=${PKG_CONFIG_PATH}:${IPOPT_DIR}/lib/pkgconfig
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${IPOPT_DIR}/lib
ENV PATH=${PATH}:${IPOPT_DIR}/lib

RUN apt-get update -y && apt-get install -y libassimp-dev
# RUN apt-get update -y && apt-get install -y coinor-libipopt-dev
COPY requirements.txt requirements.txt
RUN pip install -r requirements.txt

#Installing ADAM 
RUN git clone --depth=1 --branch humanoidsResults https://github.com/CarlottaSartore/ADAM.git
RUN cd ADAM && pip install --no-deps . && cd ..
 
# # Installing urdf-modifier 
# # RUN conda install -c conda-forge idyntree
RUN git clone --depth=1 --branch v0.0.1 https://github.com/icub-tech-iit/urdf-modifiers
RUN cd urdf-modifiers && pip install . && cd .. 
ENV DISPLAY :99
ENV RESOLUTION 1366x768x24

COPY . .
WORKDIR /src
# RUN  python3 src/eCub_lenghtAndDensityWithHumanOptimization.py
CMD [ "python3", "eCub_lenghtAndDensityWithHumanOptimization.py"]
# CMD [ "/bin/bash" ]
