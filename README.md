# Desafio Maker - Carro Robótico
![image](https://user-images.githubusercontent.com/97037684/149604070-1bfaf19c-3a3c-4108-98b0-a8a49d0561d1.png)
![image](https://user-images.githubusercontent.com/97037684/149604076-67453c9a-c9c4-4bbc-b0cb-25bd0cdb1306.png)

## Materiais/Componentes usados

![image](https://user-images.githubusercontent.com/97037684/149604175-fb236c19-ee21-405c-b9de-1eab942d8d0d.png)

- Arduino UNO
  - Documentação: https://www.circuito.io/blog/arduino-uno-pinout/
  - Datasheet: https://www.arduino.cc/en/uploads/Main/arduino-uno-schematic.pdf

- Shield SIGE Robótica
  - Datasheet: [Clique aqui para baixar o PDF](datasheet_sige_robotica.pdf)

- L298n
  - Tutorial: https://www.filipeflop.com/blog/motor-dc-arduino-ponte-h-l298n/   



## Esquema Elétrico e orientações de montagem
![image](https://user-images.githubusercontent.com/97037684/149604223-a0a32de0-d3a2-45de-a997-0319788333da.png)
![image](https://user-images.githubusercontent.com/97037684/149604231-b64e09b4-c40a-4fef-8f12-3d917d7c71d8.png)
![image](https://user-images.githubusercontent.com/97037684/149604240-f92132a0-8661-4748-9aff-4b07992f405a.png)
![image](https://user-images.githubusercontent.com/97037684/149604247-1a0c69ee-d5c0-4d91-92a2-6da4d3aa5a29.png)
![image](https://user-images.githubusercontent.com/97037684/149604256-24bf69c9-70f6-4ade-b419-6894b9b724d2.png)
![image](https://user-images.githubusercontent.com/97037684/149604269-2691e9c5-c865-4563-a619-1f75e7233efc.png)
![image](https://user-images.githubusercontent.com/97037684/149604277-052b5311-b294-4467-b707-1a2ad62af2ac.png)
![image](https://user-images.githubusercontent.com/97037684/149604285-65a1c755-34a5-4c1b-bade-8cd88c26ff83.png)
![image](https://user-images.githubusercontent.com/97037684/149604292-a1d23711-fa65-4297-9c71-d38bf523d541.png)
![image](https://user-images.githubusercontent.com/97037684/149604297-3d2094f3-52f2-48ee-a243-8a26a022d99c.png)
![image](https://user-images.githubusercontent.com/97037684/149604306-0b9550f2-755e-4c50-a47a-aacae0f2a43f.png)
![image](https://user-images.githubusercontent.com/97037684/149604310-757fc2de-12f4-42d0-af68-24ad4580e187.png)
![image](https://user-images.githubusercontent.com/97037684/149604316-5f62bd6c-f99c-4fe1-a8e3-a50be3000e15.png)

## Software/Programadores/Lib
![image](https://user-images.githubusercontent.com/97037684/149604328-18ccbfb7-d2e7-46ae-af7a-60322ae77f92.png)
![image](https://user-images.githubusercontent.com/97037684/149604340-1b87d677-1190-46c4-b149-4aad75e5c989.png)

Em
## Bibliotecas (lib) PWMservo
![image](https://user-images.githubusercontent.com/97037684/149604394-66f461a2-93ca-4992-b888-486ab7280d6b.png)
Em
## Mapeamento de portas usadas no projeto
![image](https://user-images.githubusercontent.com/97037684/149604501-6c3d0ab1-54dc-4418-8859-51659ea783ee.png)

## Código Arduino


```cpp
// Desafio Maker carro Robô
// Grupo de estudo Nova Lima
//**********************************************************************************
#include <Arduino.h>
#include <PWMServo.h>
//**********************************************************************************
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

// criando o objeto servo
PWMServo giroOlhos;
//**********************************************************************************
//variáveis
int distancia = 0; // registra a distância lida pelo sensor
int distanciaOlhoFrente = 1; // registra a distância lida pelo sensor quando andando a frente
int distanciaOlhoDireita = 1; // registra a distância lida pelo sensor quando olhando direira
int distanciaOlhoEsquerda = 1; // registra a distância lida pelo sensor quando olhando esquerda
int tempoGastoNoGiroServo = 400; // usada para configurar o tempo que a cabeça leva para girar na posição configurada
unsigned int tempoEmRe = 300; // usada para definir o tempo que vai ficar andando de ré caso o obstáculo esteja muito perto
unsigned int PrevisaotempoEmRe = millis(); // variável de contagem de tempo
unsigned int tempoVirarRobo = 400; // usada para definir o tempo que vai acionar os motores ao virar
unsigned int PrevisaotempoVirarRobo = millis(); // variável de contagem de tempo
int distanciaParaVirar = 25; // usada para configurar a distancia mínima para virar do obstáculo a frente e se manter a frente
int distanciaParaVoltar = 18; // usada para configurar a distancia mínima do obstáculo e precisa voltar
int filtro = 1; // usada para filtra e não processar valores de distancia menor que o valor configurado
bool habilitaVerQWuina = 0; // usada para habilitar a leitura de quina
bool dedoNosensor = 0; // usada para registrar se esta com o dedo no sensor LDR ou não
int verQuina = 0; // usada para registro da posição de visualização de quina
int luzAmbiente=0; // registra quantidade luz ambiente no LDR
int refLDR=0;  // valor minimo de luz tampado com o dedo
unsigned int tempoVerQuina = 300; // tempo de leitura do giro servo de quina
unsigned int PrevisaotempoVerQuina = millis(); // variável de contagem de tempo
int anguloQuinaEsq = 110; // ângulo de giro da quina esquerda
int anguloQuinaDir = 50; // ângulo de giro da quina direita
int angulocentral = 85; // ângulo de giro central
int anguloVerEsquerda = 170; // ângulo de giro da  esquerda
int anguloVerDireita = 10; // ângulo de giro da  direita
int mediaDosPot = 0; // usado para calcular a média dos potenciometros
bool podeAndarFrente=1;//  verifica se pode ir a frente
bool podeVirar=1;  // verifica se pode virar
int pwmMin=100; // valor min velocidade pwm
int pwmMax=240;  // valor max velocidade pwm
int tempViraRoboMin=200; // min tempo para virar
int tempViraRoboMax=400;  // max tempo para virar
int tempVoltarRoboMin=200;   // min tempo para voltar
int tempVoltarRoboMax=400;  // min tempo para voltar
int distVirarMin =20;  // distância min para virar
int distVirarMax =35;  // distância max para virar
int distVoltarMin =15;  // distância min para voltar
int distVoltarMax =20;  // distância max para voltar

//***********************************************************************************
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
  for (size_t i = 0; i < 100; i++)  // 100 leituras de luz
  {
    luzAmbiente+=analogRead(A0); // carrega luz na variável
    delay(10); // tempo de leitura
  }
  luzAmbiente=luzAmbiente/100; // média
  refLDR=luzAmbiente/8; // referência para o dedo no botão
}


//************************************************************************************
void loop() // programa do loop
{
  //ajustes de iluminação do botão maker
  for (size_t i = 0; i < 100; i++)
  {
    luzAmbiente+=analogRead(A0);
  }
  luzAmbiente=luzAmbiente/100; //média
  int valPot1 = analogRead(A1); // lendo o potenciômetro da esquerda e registrando na memória
  int valPot2 = analogRead(A2); // lendo o potenciômetro da direita e registrando na memória
  valPot1 = map(valPot1, 0, 1023, pwmMin, pwmMax); // transformando o valor lido no Pot1 entre 0 a 1023 e fazendo o proporcional entre 80 a 254
  valPot2 = map(valPot2, 0, 1023, pwmMin, pwmMax); // transformando o valor lido no Pot2 entre 0 a 1023 e fazendo o proporcional entre 80 a 254
  mediaDosPot = (valPot1+valPot2)/2; // média dos pwm dos motres
  tempoVirarRobo = map(mediaDosPot, pwmMin, pwmMax, tempViraRoboMax, tempViraRoboMin); // variaçao do tempo para virar em relaçao a velocidade
  tempoEmRe= map(mediaDosPot, pwmMin, pwmMax, tempVoltarRoboMax, tempVoltarRoboMin); // variaçao do tempo para Ré em relaçao a velocidade
  distanciaParaVoltar = map(mediaDosPot, pwmMin, pwmMax, distVoltarMin, distVoltarMax); // variaçao da distancia para Ré em relaçao a velocidade
  distanciaParaVirar = map(mediaDosPot, pwmMin, pwmMax, distVirarMin, distVirarMax); // variaçao da distancia para virar em relaçao a velocidade
  analogWrite(5, valPot1); // escrevendo na porta pwm5 o valor calculado para ajuste de velocidade do motor esquerdo
  analogWrite(6, valPot2); // escrevendo na porta pwm6 o valor calculado para ajuste de velocidade do motor direito
  distancia = lendoDistancia(3, 2); // variável distância registra o valor lido pelo sensor (grito no D3 e escuta o echo no D2)
  if (distancia > filtro) // filtro para eliminar falsas leitura 
  {
    distanciaOlhoFrente = distancia; // distancia olho pra frente recebe o valor lido da distância                              
  }else distancia=40; // valores falsos 
  if((millis() - PrevisaotempoVirarRobo) > tempoVirarRobo) // evitar delay nos giros do motor
  {
    podeVirar=1; // libera andar e analisar
  }else podeVirar=0; // nao anda a frente
  if ((millis() - PrevisaotempoEmRe) > tempoEmRe) // evitar delay na ré do motor
  {
    podeAndarFrente=1; // libera andar a frente
  }else podeAndarFrente=0; // nao anda a frente
   // se a distancia frente > distancia mínima para virar e andar frente e virar ok?
  if ((distanciaOlhoFrente > distanciaParaVirar)&&(podeAndarFrente==1)&&(podeVirar==1))
  {
     andarFrente(); // andar frente
  }
  else // se não, quer dizer é menor que a distancia mínima
  {
    // se a distância olho frente for menor que a mínima e precisa dar ré  
    if (distanciaOlhoFrente < distanciaParaVoltar) 
    {
      andarTras(); // andar de ré
      PrevisaotempoEmRe=millis(); // registro de tempo em ré
    }
    else // se não, quer dizer é maior que min para voltar mas ainda é menor que min para andar
    {
    if(podeVirar==1) // verifica se pode virar se outra açao não está em processo
      {
      pararRobo(); // para o robô
      MovimentoTesteDistancia(); // função de testes distância 
       // comparação da maior dist
      if (((distanciaOlhoDireita) >= (distanciaOlhoEsquerda)))
      {
        virarDireita(); // vira a direita
        PrevisaotempoVirarRobo =millis(); // registor de tempo virar
      }
      else // se nao
      {
        virarEsquerda();// vira a esquerda
        PrevisaotempoVirarRobo =millis(); // registor de tempo virar
       
      }
      giroOlhos.write(85); // sensor a frente
      verQuina = 0;        // inicializa as etapas da quina quando habilitado
     }

    }
  }
  if (luzAmbiente < refLDR) // teste do botão maker de luz
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
//**************************************************************************************
// Funções
// sensor ultrassônico
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
//****************************************************************************************
// acionando os motores na lógica para frente
void andarFrente()
{
  digitalWrite(18, 0); // andar frente
  digitalWrite(19, 1);
  digitalWrite(4, 1);
  digitalWrite(7, 0);
}
//*****************************************************************************************
// acionando os motores na lógica para trás
void andarTras()
{
  digitalWrite(18, 1);
  digitalWrite(19, 0);
  digitalWrite(4, 0);
  digitalWrite(7, 1);
}
//******************************************************************************************
// acionando os motores na lógica para parar
void pararRobo()
{
  digitalWrite(18, 0);
  digitalWrite(19, 0);
  digitalWrite(4, 0);
  digitalWrite(7, 0);
}
//*******************************************************************************************
// acionando os motores na lógica para virar a direita
void virarDireita()
{
  digitalWrite(18, 0);
  digitalWrite(19, 0);
  digitalWrite(4, 1);
  digitalWrite(7, 0);
}
//*********************************************************************************************
// acionando os motores na lógica para virar a esquerda
void virarEsquerda()
{
  digitalWrite(18, 0);
  digitalWrite(19, 1);
  digitalWrite(4, 0);
  digitalWrite(7, 0);
}
//********************************************************************************************
// função para realizar os movimentos de teste de distância direita e esquerda
void MovimentoTesteDistancia()
{
  giroOlhos.write(anguloVerDireita); // vira o olho para direita
  delay(tempoGastoNoGiroServo);
  int testeDistanciaOlhoDireita =lendoDistancia(3, 2);
  if (distancia> filtro)
  {
    distanciaOlhoDireita =testeDistanciaOlhoDireita;
  }
  giroOlhos.write(anguloVerEsquerda); // vira o olho para esquerda
  delay(tempoGastoNoGiroServo);

  int TesteDistanciaOlhoEsquerda = lendoDistancia(3, 2);
  if (distancia> filtro)
  {
    distanciaOlhoEsquerda = TesteDistanciaOlhoEsquerda;
  }
  giroOlhos.write(85);
  delay(tempoGastoNoGiroServo);
 
}
//*******************************************************************************************
// função para leitura de quina andando a frente
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

//Fim muito Obrigado
```

## Equipamentos Adicionais


01 Capacitor eletrolítico 680uf 25v em paralelo com a alimentação do servo motor, saindo da L298n
02 Capacitore cerâmicos 100nf 50v em paralelo com cada motor do robô




