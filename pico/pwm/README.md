# PWM com RP2040

## Como funciona PWM

Pulse Width Modulation (PWM), siginifica Modulação por Largura de Pulso. O mesmo consiste em pulsar rapidamente um sinal digital em um condutor de modo que simule um sinal analógico.
Em suma, é definido a largura do pulso que terá uma frequência determinada alta de modo que a porcentagem dessa largura em relação ao seu valor máximo (Duty cycle) definirá o valor de tensão de saida.

Abaixo a demonstração da matemática que define a tensão de pwm de saída sendo:
* $V_{PWM}$: Tensão de saída [V]
* $A$: Tensão do sinal digital [V]
* $D$: Duty cyle [%]
* $T_{ON}$: Período do pulso que o sinal será de nivel alto
* $T_{PERIOD}$: Período total do pulso

\[ V_{PWM} = A \cdot D \]
\[ D = \frac{T_{ON}}{T_{PERIOD}} \]

<img src="https://www.citisystems.com.br/wp-content/uploads/2016/11/modulacao-sinal-pwm.png"
     style="display: block; margin-left: auto; margin-right: auto"/>

## Funcionamento na Raspberry Pi Pico

As funções base do C/C++ SDK disponibilizado permitem fazer o controle do PWM da seguinte forma:

* O PWM Counter do microcontrolador tem uma frequência de 125 MHz e um período de 8 ns. Além disso ele tem um intervalo de [0; 65535] períodos. 
* Com essa informação temos que definir 2 parâmetros para controlar a $V_{PWM}$:
    * Wrap point: Ele definirá o ponto de quebra no PWM counter para considerado para enviar o sinal PWM  a partir de um $T_{PERIOD}$ escolhido (Ex: $T_{PERIOD}$ = 4 us, wrap point = 4 us / 8 ns = 500 Ciclos);
    * Set point: Definirá quantos ciclos do wrap point serão em nível lógico alto, obtendo o valor de $T_{ON}$ (Ex: D = 50% --> $T_{ON}$ = $D \cdot T_{PERIOD} = 50\% \cdot 500 = 250$ ciclos);

No caso da Pico, o valor de A vale 3.3V, assim a partir do duty cycle ($D$) escolhido, podemos obter uma tensão de saída variando entre [0; 3.3]V.
