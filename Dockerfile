# syntax=docker/dockerfile:1

ARG VARIANT="focal"
FROM mcr.microsoft.com/vscode/devcontainers/base:0-${VARIANT}
ARG USERNAME=user
ARG USERID=1001
ARG GIT_USER=$USERNAME
ARG GIT_USEREMAIL=$USERNAME@$USERNAME.$USERNAME
SHELL ["/bin/bash", "-c"]
ARG MINIFORGE_NAME=Miniforge3
ARG MINIFORGE_VERSION=22.9.0-2
ARG TARGETPLATFORM

ENV CONDA_DIR=/opt/conda
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
ENV PATH=${CONDA_DIR}/bin:${PATH}
# Installing conda as per https://github.com/conda-forge/miniforge-images/blob/master/ubuntu/Dockerfile
RUN apt-get update > /dev/null && \
    apt-get install --no-install-recommends --yes \
        wget bzip2 ca-certificates \
        git \
        tini \
        > /dev/null && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* && \
    wget --no-hsts --quiet https://github.com/conda-forge/miniforge/releases/download/${MINIFORGE_VERSION}/${MINIFORGE_NAME}-${MINIFORGE_VERSION}-Linux-$(uname -m).sh -O /tmp/miniforge.sh && \
    /bin/bash /tmp/miniforge.sh -b -p ${CONDA_DIR} && \
    rm /tmp/miniforge.sh && \
    conda clean --tarballs --index-cache --packages --yes && \
    find ${CONDA_DIR} -follow -type f -name '*.a' -delete && \
    find ${CONDA_DIR} -follow -type f -name '*.pyc' -delete && \
    conda clean --force-pkgs-dirs --all --yes  && \
    echo ". ${CONDA_DIR}/etc/profile.d/conda.sh && conda activate base" >> /etc/skel/.bashrc && \
    echo ". ${CONDA_DIR}/etc/profile.d/conda.sh && conda activate base" >> ~/.bashrc

ENTRYPOINT ["tini", "--"]
CMD [ "/bin/bash" ]

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

#Get coinbrew
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


# Creating the environment 
RUN conda create --name conda_env python=3.8 idyntree pip casadi numpy=1.22 pyparsing
SHELL ["conda", "run", "-n", "conda_env", "/bin/bash", "-c"]
RUN  pip install urdf-parser-py
RUN  conda install -c conda-forge python=3.8
RUN  conda install -c conda-forge pip
RUN  conda install -c conda-forge casadi
RUN  conda install -c conda-forge numpy=1.22
RUN  conda install -c conda-forge pyparsing
RUN  pip install urdf-parser-py

RUN conda install -c conda-forge irrlicht
RUN  conda install -c conda-forge idyntree
#Installing ADAM 
RUN git clone --depth=1 --branch humanoidsResults https://github.com/CarlottaSartore/ADAM.git
RUN cd ADAM && pip install . && cd ..

 
# Installing urdf-modifier 
# RUN conda install -c conda-forge idyntree
RUN git clone --depth=1 --branch v0.0.1 https://github.com/icub-tech-iit/urdf-modifiers
RUN cd urdf-modifiers && pip install . && cd .. 
ENV DISPLAY :99
ENV RESOLUTION 1366x768x24

COPY . .
# RUN conda install -c conda-forge python=3.8
RUN  python3 src/eCub_lenghtAndDensityWithHumanOptimization.py

