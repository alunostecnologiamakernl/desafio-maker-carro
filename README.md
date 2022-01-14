# Desafio Maker - Carro Robótico


![image](https://user-images.githubusercontent.com/97037684/149436288-0bd3f940-cd03-4864-8060-44fff54319f0.png)
![image](https://user-images.githubusercontent.com/97037684/149436402-5dbfe703-0b52-4b14-bed6-3f454d6e9b7b.png)

## Materiais/Componentes usados

![image](https://user-images.githubusercontent.com/97037684/149436690-402c0864-398c-4b01-a1e7-bad040ecfd35.png)
- Arduino UNO
  - Documentação: https://www.circuito.io/blog/arduino-uno-pinout/
  - Datasheet: https://www.arduino.cc/en/uploads/Main/arduino-uno-schematic.pdf

- Shield SIGE Robótica
  - Datasheet: [Clique aqui para baixar o PDF](datasheet_sige_robotica.pdf)

- L298n
  - Tutorial: https://www.filipeflop.com/blog/motor-dc-arduino-ponte-h-l298n/   



## Esquema Elétrico e orientações de montagem

![image](https://user-images.githubusercontent.com/97037684/149436772-2809fafa-9e39-4d2b-9918-075e6aa630e5.png)
![image](https://user-images.githubusercontent.com/97037684/149436786-d5b94fce-2d0c-40e1-a09d-0546730b3106.png)
![image](https://user-images.githubusercontent.com/97037684/149436803-855ac2b5-73c9-4de7-84a4-25a5b1135efe.png)
![image](https://user-images.githubusercontent.com/97037684/149436811-6fd6fa00-c7d1-4bd5-a44b-f6dbf22fc123.png)
![image](https://user-images.githubusercontent.com/97037684/149436819-3190cc94-4adc-4ce6-9cc3-5ef9921d07b4.png)
![image](https://user-images.githubusercontent.com/97037684/149436964-51c4ed35-2a64-4d23-897a-8470c159cb8c.png)
![image](https://user-images.githubusercontent.com/97037684/149437055-976a6408-906b-4656-a211-0beb9631cda2.png)
![image](https://user-images.githubusercontent.com/97037684/149437139-b3d0852b-5a05-4ebc-bf9d-e2d760dd73fc.png)
![image](https://user-images.githubusercontent.com/97037684/149437188-61e7e1e3-c319-471a-b88e-93e90858aecc.png)
![image](https://user-images.githubusercontent.com/97037684/149437214-9261f60e-8a61-4efe-a1ac-dace2a6fdaf8.png)
![image](https://user-images.githubusercontent.com/97037684/149437234-6eb1792a-b3e1-46b5-9266-c823821d3c7b.png)
![image](https://user-images.githubusercontent.com/97037684/149437379-b28c8b50-e0df-481a-b2f2-5de1aae07346.png)
![image](https://user-images.githubusercontent.com/97037684/149437395-786a5146-7b26-4aee-a01f-b2f8c3379b25.png)
![image](https://user-images.githubusercontent.com/97037684/149437414-c641f3f4-bb95-4f69-ae03-deba370d97b8.png)





## Software/Programadores/Lib

![image](https://user-images.githubusercontent.com/97037684/149438975-e19ec5f9-556a-4a13-8fd8-1e28da2f949c.png)
![image](https://user-images.githubusercontent.com/97037684/149439003-7a28e03c-7d6a-49c8-bcac-dda701425785.png)


## Bibliotecas (lib) PWMservo

![image](https://user-images.githubusercontent.com/97037684/149439122-99ef5a1a-8078-4b90-b4a8-adf987f1756c.png)
Baixar aqui:  https://www.arduino.cc/reference/en/libraries/pwmservo/


## Mapeamento de portas usadas no projeto

![image](https://user-images.githubusercontent.com/97037684/149439390-79dab4dd-f7c0-45e5-9980-f57f2634adf5.png)



## Código Arduino


```cpp
// Desafio Maker carro Robô
// Grupo de estudo Nova Lima

#include <Arduino.h>
#include <PWMServo.h>
//*********************************************************************************************
// Declarações funções:

// Função para ler a distância
int lendoDistancia(int trigPin, int echoPin);
// Função para andar a frente
void andarFrente();
void andarTras();               // Função para andar de ré
void pararRobo();               // Função para parar o robô
void virarDireita();            // Função para virar a direita
void virarEsquerda();           // Função para virar a Esquerda
void MovimentoTesteDistancia(); // Função de giro da cabeça para análise dos obstáculos
void LerQuina();                // Função para habilitar a varredura da quina
//*********************************************************************************************
// criando o objeto servo

PWMServo giroOlhos;


//*********************************************************************************************
//Variáveis

double distancia = 0.0; // registra a distância lida pelo sensor
double distanciaOlhoFrente = 0.0; // registra a distância lida pelo sensor quando andando a frente
double distanciaOlhoDireita = 0.0; // registra a distância lida pelo sensor quando olhando direira
double distanciaOlhoEsquerda = 0.0; // registra a distância lida pelo sensor quando olhando esquerda
int tempoGastoNoGiroServo = 500; // usada para configurar o tempo que a cabeça leva para girar na posição configurada
int tempoEmRe = 300; // usada para definir o tempo que vai ficar andando de ré caso o obstáculo esteja muito perto
int tempoVirarRobo = 300; // usada para definir o tempo que vai acionar os motores ao virar
double distanciaParaVirar = 20.0; // usada para configurar a distancia mínima para virar do obstáculo a frente e se manter a frente
double distanciaParaVoltar = 15.0; // usada para configurar a distancia mínima do obstáculo e precisa voltar
double filtro = 1.00; // usada para filtra e não processar valores de distancia menor que o valor configurado
bool habilitaVerQWuina = 1; // usada para habilitar a leitura de quina
bool dedoNosensor = 0; // uada para registrar se esta com o dedo no sensor LDR ou não
int verQuina = 0; // usada para registro da posição de visualização de quina
unsigned int tempoVerQuina = 500; // tempo de leitura do giro servo de quina
unsigned int PrevisaotempoVerQuina = millis(); // variável de contagem de tempo
int anguloQuinaEsq = 100; // angulo de giro da quina esquerda
int anguloQuinaDir = 60; // angulo de giro da quina direita
unsigned long _ABVAR_1_tempo = 0UL ;

//**************************************************************************************************
void setup() // configurações iniciais
{

  giroOlhos.attach(SERVO_PIN_A); // inicializa o servo no pin 9
  digitalWrite(3, 0); // inicializa o pino grito com 0 desligado
  Serial.begin(9600); // inicializa a serial
  // definindo os 02 pinos digitais de controle do motor da direitra como saída
  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);
  // definindo os 02 pinos digitais de controle do motor da esquerda como saída
  pinMode(4, OUTPUT);
  pinMode(7, OUTPUT);
  // posicionando o servo motor em aproximadamente 90 graus para frente
  giroOlhos.write(85);
  // aguardando 1s para iniciar o robô
  delay(1000);
  // _ABVAR_1_tempo = 0UL ;
}
//**************************************************************************************************

void loop() // programa do loop
{
  float valPot1 = analogRead(A1); // lendo o potenciômetro da esquerda e registrando na memória
  float valPot2 = analogRead(A2); // lendo o potenciômetro da direita e registrando na memória

  valPot1 = map(valPot1, 0, 1023, 80, 254); // transformando o valor lido no Pot1 entre 0 a 1023 e fazendo o proporcional entre 80 a 254
  valPot2 = map(valPot2, 0, 1023, 80, 254); // transformando o valor lido no Pot2 entre 0 a 1023 e fazendo o proporcional entre 80 a 254

  // Serial.print("val:  ");
  // Serial.println(val);

  analogWrite(5, valPot1); // escrevendo na porta pwm5 o valor calculado para ajuste de velocidade do motor esquerdo
  analogWrite(6, valPot2); // escrevendo na porta pwm6 o valor calculado para ajuste de velocidade do motor direito

  distancia = lendoDistancia(3, 2); // variável distância registra o valor lido pelo sensor (grito no D3 e escuta o echo no D2)

  if (distancia > 0.0) // filtro para eliminar falsas leitura de 0
  {
    distanciaOlhoFrente = distancia; // distancia olho pra frente recebe o valor lido da distância
                                     //    Serial.print("dist");
                                     //    Serial.print(" ");
                                     //    Serial.print(distanciaOlhoFrente);
                                     //    Serial.print(" ");
                                     //    Serial.println();
  }
  if (((distanciaOlhoFrente) > (distanciaParaVirar))) // se a distancia olho frente for maior que a distancia mínima para virar?
  {
    andarFrente(); // andar frente
  }
  else // se não, quer dizer é menor que a distancia mínima
  {
    if (distanciaOlhoFrente < distanciaParaVoltar) // se a distância olho frente for menor que a mínima e precisa dar ré
    {
      andarTras(); // andar de ré
    }
    else // se não, quer dizer é maior que 15cm mas ainda é menor que 25cm
    {
      pararRobo(); // para o robô

      MovimentoTesteDistancia(); // função de testes distância

      if (((distancia) > (filtro))) // teste do filtro
      {
        distanciaOlhoEsquerda = distancia; // registro dist a esquerda
      }
      if (((distanciaOlhoDireita) >= (distanciaOlhoEsquerda))) // comparação da maior dist
      {
        virarDireita();                                       // vira a direita
        if (((distanciaOlhoDireita) < (distanciaParaVoltar))) // teste após ação se chegou muito perto
        {
          andarTras(); // anda de ré
        }
      }
      else
      {
        virarEsquerda();                                       // vira a esquerda
        if (((distanciaOlhoEsquerda) < (distanciaParaVoltar))) // teste após ação se chegou muito perto
        {
          andarTras(); // anda de ré
        }
      }
      giroOlhos.write(85); // sensor a frente
      verQuina = 0;        // inicializa as etapas da quina quando habilitado
    }
  }
  if (analogRead(A0) < 8) // teste do botão maker de luz
  {
    if (dedoNosensor == 0) // se o dedo não estava na posição
    {
      habilitaVerQWuina = !habilitaVerQWuina; // habilita e desabilita a função quina
    }
    dedoNosensor = 1; // informa que o dedo esta no sensor
    delay(500);       // tempo
  }
  else
    dedoNosensor = 0; // tirou o dedo

  if (habilitaVerQWuina == 1) // teste se a função quina esta ativa
  {
    LerQuina(); // função quina
  }
  else
    giroOlhos.write(85); // Mantem o olho parado a frente
}
//******************************************************************************************************************

// Funções

int lendoDistancia(int Dgrito, int Decho)
{
  long tempo;
  pinMode(Dgrito, OUTPUT);
  pinMode(Decho, INPUT);
  digitalWrite(Dgrito, 0);
  delayMicroseconds(2);
  digitalWrite(Dgrito, 1);
  delayMicroseconds(20);
  digitalWrite(Dgrito, 0);
  tempo = pulseIn(Decho, 1);
  tempo = tempo / 59;
  if ((tempo < 2) || (tempo > 300))
    return false;
  return tempo;
}
//******************************************************************************************************************
void andarFrente()
{
  digitalWrite(18, 0); // andar frente
  digitalWrite(19, 1);
  digitalWrite(4, 1);
  digitalWrite(7, 0);
}

//*******************************************************************************************************************
void andarTras()
{
  digitalWrite(18, 1);
  digitalWrite(19, 0);
  digitalWrite(4, 0);
  digitalWrite(7, 1);
  delay(tempoEmRe);
}

//*******************************************************************************************************************
void pararRobo()
{
  digitalWrite(18, 0);
  digitalWrite(19, 0);
  digitalWrite(4, 0);
  digitalWrite(7, 0);
}

//********************************************************************************************************************
void virarDireita()
{
  digitalWrite(18, 0);
  digitalWrite(19, 0);
  digitalWrite(4, 1);
  digitalWrite(7, 0);
  delay(tempoVirarRobo);
}

//*********************************************************************************************************************
void virarEsquerda()
{
  digitalWrite(18, 0);
  digitalWrite(19, 1);
  digitalWrite(4, 0);
  digitalWrite(7, 0);
  delay(tempoVirarRobo);
}

//**********************************************************************************************************************
void MovimentoTesteDistancia()
{
  giroOlhos.write(10); // vira o olho para direita
  delay(tempoGastoNoGiroServo);
  distancia = lendoDistancia(3, 2);
  if (((distancia) > (0.0)))
  {
    distanciaOlhoDireita = distancia;
  }
  giroOlhos.write(85); // vira o olho para o centro
  delay(tempoGastoNoGiroServo);
  giroOlhos.write(170); // vira o olho para esquerda
  delay(tempoGastoNoGiroServo);
  distancia = lendoDistancia(3, 2);
  giroOlhos.write(85);
  delay(tempoGastoNoGiroServo);
}

//***********************************************************************************************************************
void LerQuina()
{
  if ((verQuina == 0) && ((millis() - PrevisaotempoVerQuina) > tempoVerQuina))
  {
    PrevisaotempoVerQuina = millis();
    verQuina = 1;
    giroOlhos.write(anguloQuinaEsq);
  }
  else if ((verQuina == 1) && ((millis() - PrevisaotempoVerQuina) > tempoVerQuina))
  {
    PrevisaotempoVerQuina = millis();
    verQuina = 2;
    giroOlhos.write(85);
  }
  else if ((verQuina == 2) && ((millis() - PrevisaotempoVerQuina) > tempoVerQuina))
  {
    PrevisaotempoVerQuina = millis();
    verQuina = 3;
    giroOlhos.write(anguloQuinaDir);
  }
  else if ((verQuina == 3) && ((millis() - PrevisaotempoVerQuina) > tempoVerQuina))
  {
    PrevisaotempoVerQuina = millis();
    verQuina = 0;
    giroOlhos.write(85);
  }
}
```

## Equipamentos Adicionais


01 Capacitor eletrolítico 680uf 25v em paralelo com a alimentação do servo motor, saindo da L298n
02 Capacitore cerâmicos 100nf 50v em paralelo com cada motor do robô




