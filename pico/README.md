# Testes das features da Raspberry Pi Pico
---
## Informações necessárias

A plataforma escolhida para os códigos foi a C/C++ SDK fornecida pela Raspberry Pi. A mesma fornece um [tutorial](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf) de como fazer a utilização da plataforma. Em suma, para utilização da mesma deve se instalar pacotes, efetuar a configuração do ambiente, aprender a compilar códigos e carregar o arquivo produzido na Pico. 
A plataforma foi escolhida ao inves da MicroPython e Arduino por ter um aproveitamento melhor do hardware, sendo uma opção que reproduz algoritmos mais rápidos sem possuir uma semântica tão complexa (comparando com linguagens de mais baixo nível como assembly).
Por fim, [além da documentação da linguagem](https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf), a empresa disponibiliza alguns [tutoriais](https://raspberrypi.github.io/pico-sdk-doxygen/examples_page.html) de códigos dessa plataforma, os quais, serão a principal fonte para construção dos códigos de testes abaixo.

A seguir está a ordem dos comandos para fazer a istalação do C/C++ SDK, lembrando que no tutorial tem a explicação de cada passo:

```bash
$ cd ~/
$ mkdir pico
$ cd pico
$ git clone -b master https://github.com/raspberrypi/pico-sdk.git
$ cd pico-sdk
$ git submodule update --init
$ cd ..
$ mkdir pico-code
$ sudo apt update
$ sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
$ cd pico-sdk
$ git pull
$ git submodule update
```

Para fazer o build do projeto e compilar o arquivo (os códigos são criados em pastas dentro da pico-code):

```bash
$ cd pico-code
$ mkdir build
$ cd build
$ export PICO_SDK_PATH=../../pico-sdk
$ cmake ..
$ cd <pasta_código>
$ make -j4
```

Assim será gerado um arquivo com extensão .uf2, o qual deve ser carregado na Raspberry Pi Pico. Com isso agora será listado os testes.

