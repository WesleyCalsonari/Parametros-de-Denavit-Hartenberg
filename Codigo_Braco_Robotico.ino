/*
  Código: Cinemática direta com Denavit-Hartenberg
  Autor: Wesley Calsonari Nogueira
  Data: 11/06/2023
*/
#include <Servo.h>  // Inclusão da biblioteca servo

#define pinbas 3  // Define pino para controlar o servo da base
#define pindir 5  // Define pino para controlar o servo da direita
#define pinesq 6  // Define pino para controlar o servo da esquerda

Servo base, dir, esq;   // Cria objetos do tipo servo
unsigned char num = 0;  // Cria variável auxiliar para a execução dos passos no código

// Parâmetros de Denavit-Hartenberg
double t1, t2, t3;                  // Ângulos theta em graus
double d1 = 1.3, d2 = 0, d3 = 0;    // Valores de deslocamento ao longo do eixo z
double a1 = 0, a2 = 3.5, a3 = 8;    // Valores de deslocamento ao longo do eixo x
double af1 = 90, af2 = 0, af3 = 0;  // Ângulos alpha em graus


void setup() {
  Serial.begin(9600);                         // Inicia a comunicação serial com baud rate de 9600
  Serial.print("Digite o angulo theta 1: ");  // Escreve uma mensagem no monitor serial
  base.attach(pinbas);                        // Associação do pino digital ao objeto do tipo servo
  dir.attach(pindir);                         // Associação do pino digital ao objeto do tipo servo
  esq.attach(pinesq);                         // Associação do pino digital ao objeto do tipo servo
}

// Função que converte o ângulo de graus para radianos
double degtorad(double ang) {
  return ((ang / 180) * (PI));
}

void loop() {
  // Trecho desenvolvido para receber os ângulos theta 1, theta 2 e theta 3 em graus
  if (Serial.available()) {
    switch (num) {
      case 0:
        t1 = Serial.readString().toInt();           // Converte o valor recebido para o tipo inteiro e armazena em t1
        Serial.println(t1);                         // Escreve uma mensagem no monitor serial
        Serial.print("Digite o angulo theta 2: ");  // Escreve uma mensagem no monitor serial
        num++;                                      // Incrementa a variável num
        break;                                      // Quebra o fluxo do switch
      case 1:
        t2 = Serial.readString().toInt();           // Converte o valor recebido para o tipo inteiro e armazena em t2
        Serial.println(t2);                         // Escreve uma mensagem no monitor serial
        Serial.print("Digite o angulo theta 3: ");  // Escreve uma mensagem no monitor serial
        num++;                                      // Incrementa a variável num
        break;                                      // Quebra o fluxo do switch
      case 2:
        t3 = Serial.readString().toInt();  // Converte o valor recebido para o tipo inteiro e armazena em t3
        Serial.println(t3);                // Escreve uma mensagem no monitor serial
        Serial.println();                  // Escreve uma mensagem no monitor serial
        num++;                             // Incrementa a variável num
        break;                             // Quebra o fluxo do switch
    }
  }

  // Trecho desenvolvido para calcular a matriz de transformação direta e movimentar o braço robótico
  if (num == 3) {
    // Define matriz A_1
    double A_1[4][4] = { { cos(degtorad(t1)), (-sin(degtorad(t1))) * cos(degtorad(af1)), sin(degtorad(t1)) * sin(degtorad(af1)), a1 * cos(degtorad(t1)) },
                         { sin(degtorad(t1)), cos(degtorad(t1)) * cos(degtorad(af1)), (-cos(degtorad(t1))) * sin(degtorad(af1)), a1 * sin(degtorad(t1)) },
                         { 0, sin(degtorad(af1)), cos(degtorad(af1)), d1 },
                         { 0, 0, 0, 1 } };

    // Define matriz A_2
    double A_2[4][4] = { { cos(degtorad(t2)), (-sin(degtorad(t2))) * cos(degtorad(af2)), sin(degtorad(t2)) * sin(degtorad(af2)), a2 * cos(degtorad(t2)) },
                         { sin(degtorad(t2)), cos(degtorad(t2)) * cos(degtorad(af2)), (-cos(degtorad(t2))) * sin(degtorad(af2)), a2 * sin(degtorad(t2)) },
                         { 0, sin(degtorad(af2)), cos(degtorad(af2)), d2 },
                         { 0, 0, 0, 1 } };

    // Define matriz A_3
    double A_3[4][4] = { { cos(degtorad(t3)), (-sin(degtorad(t3))) * cos(degtorad(af3)), sin(degtorad(t3)) * sin(degtorad(af3)), a3 * cos(degtorad(t3)) },
                         { sin(degtorad(t3)), cos(degtorad(t3)) * cos(degtorad(af3)), (-cos(degtorad(t3))) * sin(degtorad(af3)), a3 * sin(degtorad(t3)) },
                         { 0, sin(degtorad(af3)), cos(degtorad(af3)), d3 },
                         { 0, 0, 0, 1 } };
    double MA1_A2[4][4];  // Define matriz MA1_A2
    double AT[4][4];      // Define a matriz (AT) de tranformação direta
    double somaprod;      // Cria variável auxiliar

    // Calcula o produto de A1 e A2
    int M1L = 4, M1C = 4, M2L = 4, M2C = 4;
    for (int linha = 0; linha < M1L; linha++)
      for (int coluna = 0; coluna < M2C; coluna++) {
        somaprod = 0;
        for (int i = 0; i < M1L; i++) somaprod += A_1[linha][i] * A_2[i][coluna];
        MA1_A2[linha][coluna] = somaprod;
      }

    // Calcula o produto de A1*A2 com A3
    for (int linha = 0; linha < M1L; linha++)
      for (int coluna = 0; coluna < M2C; coluna++) {
        somaprod = 0;
        for (int i = 0; i < M1L; i++) somaprod += MA1_A2[linha][i] * A_3[i][coluna];
        AT[linha][coluna] = somaprod;
      }

    Serial.println("\n");  // Pula linhas no monitor serial

    //Imprime o produto das Matrizes A1, A2 e A3
    for (int linha = 0; linha < M1L; linha++) {
      for (int coluna = 0; coluna < M2C; coluna++) {
        Serial.print(AT[linha][coluna]);
        Serial.print(" ");
      }
      Serial.println("\n");  // Pula linhas no monitor serial
    }
    base.write(t1);                             // Escreve o ângulo t1 no servo da base
    dir.write(t2);                              // Escreve o ângulo t2 no servo da direita
    esq.write(180 - t3);                        // Escreve o ângulo t3 no servo da esquerda (OBS: Fazemos esta diferença devido a inversão do sentido de giro do servo no braço)
    delay(100);                                 // Aguarda 1ms
    num = 0;                                    // Atribui valor 0 a variável num
    Serial.print("Digite o angulo theta 1: ");  // Escreve uma mensagem no monitor serial
  }
}
