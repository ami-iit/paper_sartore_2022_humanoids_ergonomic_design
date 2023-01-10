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

# Installing Anaconda 
# Use the above args during building https://docs.docker.com/engine/reference/builder/#understand-how-arg-and-from-interact\
ARG UBUNTU_VER=20.04
ARG CONDA_VER=latest
ARG OS_TYPE=x86_64

# Install miniconda to /miniconda
RUN curl -LO "http://repo.continuum.io/miniconda/Miniconda3-${CONDA_VER}-Linux-${OS_TYPE}.sh"
RUN bash Miniconda3-${CONDA_VER}-Linux-${OS_TYPE}.sh -p /miniconda -b
RUN rm Miniconda3-${CONDA_VER}-Linux-${OS_TYPE}.sh
ENV PATH=/miniconda/bin:${PATH}
RUN conda update -y conda
RUN conda init bash 
RUN source ~/.bashrc
RUN conda create --name humanoids_results

RUN echo "conda activate humanoids_results" > ~/.bashrc
RUN conda install -c conda-forge idyntree
RUN conda install -c conda-forge casadi
RUN conda install -c conda-forge pip
RUN conda install -c conda python=3.8
# Install essentials
COPY requirements.txt requirements.txt
RUN pip install -r requirements.txt

# Create the user
RUN useradd --create-home -s /bin/bash --no-user-group -u $USERID $USERNAME && \
    adduser $USERNAME sudo && \
    echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

USER $USERNAME
WORKDIR /home/$USERNAME
# Get coinbrew
COPY coinhsl.zip . 
RUN mkdir CoinIpopt &&\
    cd CoinIpopt &&\
    wget https://raw.githubusercontent.com/coin-or/coinbrew/1cd6e53a9c6f3b235939da3ebf6cabaa14da44bd/coinbrew &&\
    chmod u+x coinbrew && \
    ./coinbrew build Ipopt --main-proj-version=releases/3.13.4 --prefix=install --test --no-prompt --verbosity=3 && \
    cd /home/$USERNAME && unzip coinhsl.zip \
    && mv coinhsl-2019.05.21 CoinIpopt/ThirdParty/HSL/coinhsl && \
    cd CoinIpopt &&\
    ./coinbrew build Ipopt --main-proj-version=releases/3.13.4 --prefix=install --test --no-prompt --verbosity=3 && \
    ./coinbrew install Ipopt --no-prompt && \
    cd install/lib && \
    ln -s libcoinhsl.so libhsl.so && \
    cd /home/$USERNAME/CoinIpopt && \
    rm -rf build

RUN echo 'export IPOPT_DIR=/home/$USERNAME/CoinIpopt/install' >> /home/$USERNAME/.bashrc && \
    echo 'export PKG_CONFIG_PATH=${PKG_CONFIG_PATH}:${IPOPT_DIR}/lib/pkgconfig' >> /home/$USERNAME/.bashrc && \
    echo 'export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${IPOPT_DIR}/lib' >> /home/$USERNAME/.bashrc && \
    echo 'export PATH=${PATH}:${IPOPT_DIR}/lib' >> /home/$USERNAME/.bashrc

RUN git config --global user.name ${GIT_USER} && \
    git config --global user.email ${GIT_USEREMAIL}
ENV IPOPT_DIR=/home/$USERNAME/CoinIpopt/install
ENV PKG_CONFIG_PATH=${PKG_CONFIG_PATH}:${IPOPT_DIR}/lib/pkgconfig
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${IPOPT_DIR}/lib
ENV PATH=${PATH}:${IPOPT_DIR}/lib

# Installing ADAM 
RUN git clone --depth=1 --branch humanoidsResults https://github.com/CarlottaSartore/ADAM.git
RUN cd ADAM && pip install . && cd ..
 
# Installing urdf-modifier 
RUN git clone --depth=1 --branch v0.0.1 https://github.com/icub-tech-iit/urdf-modifiers
RUN cd urdf-modifiers && pip install . && cd .. 
COPY . .
RUN python3 src/eCub_lenghtAndDensityWithHumanOptimization.py

