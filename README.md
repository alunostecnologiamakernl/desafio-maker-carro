



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

## Software/Programadores
![image](https://user-images.githubusercontent.com/97037684/149604328-18ccbfb7-d2e7-46ae-af7a-60322ae77f92.png)
![image](https://user-images.githubusercontent.com/97037684/149604340-1b87d677-1190-46c4-b149-4aad75e5c989.png)


## Bibliotecas (lib) PWMservo
![image](https://user-images.githubusercontent.com/97037684/149604394-66f461a2-93ca-4992-b888-486ab7280d6b.png)

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
// possíveis ajustes nas variáveis para resultados na prática
// Esse valores podem ser alterados de acordo com a bateria usada, motores, ponte H, lembrando que temos os limites de 1 a 254 para que considere alguma alteração
int pwmMin=100; // valor min velocidade pwm esse é o valor mínimo enviado com o analogwrite para o pwm  e controle de velocidade
int pwmMax=240;  // valor max velocidade pwm esse é o valor máximo enviado com o analogwrite para o pwm  e controle de velocidade

// Esses são os tempos em ms para realizar os processos de virar a esquerda e direita quando detectado obstáculos
// O auto ajuste em relaçao a velocidade e inversamente proporcional está configurado entre o ranger abaixo
// conforme sua configuração de hardware e ambiente use esses valores para configurar conforme seu cenário.
//  somente evitem tempos abaixo de 200ms para o robô não entrar em loop

//tempo para virar
int tempViraRoboMin=100; // min tempo para virar
int tempViraRoboMax=700;  // max tempo para virar

//tempo para voltar
int tempVoltarRoboMin=10;   // min tempo para voltar
int tempVoltarRoboMax=50;  // min tempo para voltar

//Ranger proporcional a velocidade para configuração dos limites de distância
// distância para analisar e fazer o giro a esquerda e direita
int distVirarMin =20;  // distância min para virar
int distVirarMax =35;  // distância max para virar

// distância para voltar, pois não vai conseguir virar
int distVoltarMin =10;  // distância min para voltar
int distVoltarMax =25;  // distância max para voltar

//Valor angular do giro da cabeça servo
//quando andando a frente e a função quina habilitado
int anguloQuinaEsq = 110; // ângulo de giro da quina esquerda
int anguloQuinaDir = 50; // ângulo de giro da quina direita

// Quando detectado um obstáculo e vai realizar o análise das direções
int angulocentral = 85; // ângulo de giro central
int anguloVerEsquerda = 170; // ângulo de giro da  esquerda
int anguloVerDireita = 10; // ângulo de giro da  direita

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
int mediaDosPot = 0; // usado para calcular a média dos potenciometros
bool podeAndarFrente=1;//  verifica se pode ir a frente
bool podeVirar=1;  // verifica se pode virar


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
  refLDR=luzAmbiente/5; // referência para o dedo no botão
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
  analogWrite(5, valPot1); // escrevendo na porta pwm5 o valor calculado para ajuste de velocidade do motor esquerdo
  analogWrite(6, valPot2); // escrevendo na porta pwm6 o valor calculado para ajuste de velocidade do motor direito
  
  mediaDosPot = (valPot1+valPot2)/2; // média dos pwm dos motres
  tempoVirarRobo = map(mediaDosPot, pwmMin, pwmMax, tempViraRoboMax, tempViraRoboMin); // variaçao do tempo para virar em relaçao a velocidade
  tempoEmRe= map(mediaDosPot, pwmMin, pwmMax, tempVoltarRoboMax, tempVoltarRoboMin); // variaçao do tempo para Ré em relaçao a velocidade
  distanciaParaVoltar = map(mediaDosPot, pwmMin, pwmMax, distVoltarMin, distVoltarMax); // variaçao da distancia para Ré em relaçao a velocidade
  distanciaParaVirar = map(mediaDosPot, pwmMin, pwmMax, distVirarMin, distVirarMax); // variaçao da distancia para virar em relaçao a velocidade
  tempoVerQuina=map(mediaDosPot, pwmMin, pwmMax, 400, 200); //variação do tempo de giro na funçao ver quina 
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
## Instruções de funcionamento
O nosso Robô maker foi montado numa plataforma de prototipagem maker, usamos a criatividade sem medir esforços para demonstrar muito mais que um robô andando e sim tomando atitudes um pouco mais inteligente no quesito desvio de obstáculos. Como o cenário físico, hardware, podem sofrer e responder um pouco diferente do nosso protótipo, os alunos do grupo de estudo do desafio Maker da cidade de Nova Lima, orientados pelo Professor Fernando Fernandes e com apoio e parceria da Secretaria de Educação de Nova Lima, resolveram inovar e tentar diminuir esses impactos, usando a lógica. Então vamos aos procedimentos:
Após realizar toda montagem conforme as instruçoes nesse tutorial, baixar a lib PWMservo, colocar na pasta libraries onde esta instalada a IDE do arduino, carregar o código do nosso projeto e enviar para a placa. Com a shield Robótica SigeControl já conectada, vamos realizar o ajuste fino de velocidade para o sincronismo dos motores, o sincronismo inversamente proporcional da velocidade em relação ao tempo de deslocamento, os limites de leitura das distâncias, nesse caso diretamente proporcionais à velocidade, para açoes na lógica digital.
Bom parece complicado não acham? Mas fiquem calmo, o complicado faz parte da nossa inovação, todos os cáculos e formas para todos esses ajustes, já estão integrados no firmware, assim a única coisa que você vai precisar fazer é garantir que as baterias estejam boas, ligar o robô Maker e soltar em um ambiente sem obstáculos a frente, observando se a roda boba está bem retinha para não atrapalhar no ajuste e observar se o robô esta andando a frente corretamente. Caso não esteja assim, observe qual o lado ele está virando, mesmo com os dois motores atuando para girar para frente, na placa SigeControl temos 02 potenciômetro nativos e ligados na A1 e A2 do arduino, gire esses potenciômetros observando que a velocidade do motor irá alterar, assim use o potenciômetro A1 para controlar a velocidade do motor da esquerda e o potenciômetro A2 para controlar o motor da direita. Use a regra de deslocamento do veículo robô de 02 motores, caso esteja indo para direita por exemplo, aumente a velocidade do motor do mesmo lado, ou diminua a velocidade do motor do sentido oposto. Assim poderá realizar um ajuste fino de velocidade, alinhando o giro do movimento para frente o melhor possível.
É inversamente proporcional, a relação da velocidade e o tempo, para acionamento das funções de direção do robô, e são automaticamente ajustadas, juntamente com os limites de distância de ostáculo para tomada de decisão. Afinal em relação a velocidade e distância, precisamos aplicar a lógica  diretamente proporcional, quanto maior a velocidade, maior deve ser os limitadores de distância para evitar colisõe sem o tempo suficiente para a parar. Então o nosso sistema está preparado com alguns padrões limites que podem ser alterados no código caso totalmente diferentes,  mas na maioria dos casos é somente ajustar a velocidade dos motores que as demais caracteristicas se ajustam.

Itens e opções influenciadas no ajuste da velocidade:
Valor analógico lido pelos potenciômetro:
Esse valor é lido e transformado diretamente para duas portas PWM para o controle de velocidade do Motor 1 e Motor 2.
Fazendo essas alterações o sistema faz uma média nos dois motores e calcula inversamente proporcional o tempo, usando como referência o millis(), para os acionamentos do giro à esquerda e direita. Inversamente, pois, quanto maior a velocidade, menor será o tempo para realizar o trabalho de virar para a posição (esquerda e direita) definida pela logica de análise do robô.
Outra inovação conforme explicado acima é nos limitadores de distância, onde temos duas referências, a miníma distância de uma objeto sólido que necessita de retorno e a mínima distância de um objeto sólido que o sistema irá decidir no processo de análise de obstáculos a esquerda e direita.
Outra coisa que fizemos foi a criação do botão maker, um botão usando o LDR nativo da SigeControl com finalidade de habilitar ou não uma função especial de leitura de quina, quando deslocando à frente sem obstáculos. O botão foi criado pois podemos ter resultados diferenciados com o uso da função ou não, conforme as formas e métodos de obstáculos usados nos testes.
Falando em obstáculos, como nosso análise de distância é calculado como a biologia de deslocamento de um Morcego, quer dizer, emite um som, aguarda o retorno do eco e sabendo a velocidade de propagação do som no meio, conseguimos calcular a distância de um obstáculo, senso assim é recomendado o uso de obstáculos sólidos e que propague bem o eco,  evitando obstáculos de tecidos, porosos, revestimentos que abafam o eco. Outras dica para testes, use montágens de obstáculos formando ângulos de 90 graus para entender melhor a inteligência do sistema embarcado com decisões para o acionamento dos motores.
 


## Equipamentos Adicionais


01 Capacitor eletrolítico 680uf 25v em paralelo com a alimentação do servo motor, saindo da L298n

02 Capacitore cerâmicos 100nf 50v em paralelo com cada motor do robô


## Fotos e vídeos do processo de desenvolvimento, resultados e aprendizado do grupo

Sala de aula usada pelo grupo de estudo
![WhatsApp Image 2022-01-06 at 15 41 56](https://user-images.githubusercontent.com/97037684/149644374-08d3fdc7-95e0-47b3-b1d1-e5a161365896.jpeg)
![WhatsApp Image 2022-01-06 at 15 42 18](https://user-images.githubusercontent.com/97037684/149645593-75ff8607-014e-485f-85b4-ebbb93dde4da.jpeg)


Suporte de elevação do robô, para ajudar nos testes no desenvolvimento da lógica
![WhatsApp Image 2022-01-12 at 12 26 45 (1)](https://user-images.githubusercontent.com/97037684/149644573-ad92dee8-2540-4e36-b361-801209458f03.jpeg)
![WhatsApp Image 2022-01-12 at 12 26 45](https://user-images.githubusercontent.com/97037684/149645612-58bdaaae-cf12-47e9-9e42-58f13c5e0529.jpeg)

Início da programação em blocos
![WhatsApp Image 2022-01-12 at 12 26 45 (3)](https://user-images.githubusercontent.com/97037684/149644629-50f46559-4e5d-4fb8-a145-9af97f2050f0.jpeg)

Os primeiros testes, e os primeiros erros, kkk

![image](https://user-images.githubusercontent.com/97037684/149644831-b7fa0ad2-4ec8-47cb-af3c-4ca81c1fc299.png)
![image](https://user-images.githubusercontent.com/97037684/149645125-70a03653-2fc9-456b-afef-3785f4e0d361.png)




https://user-images.githubusercontent.com/97037684/149645771-e457a2c1-f813-4634-add8-4c9ad5a38c7e.mp4



https://user-images.githubusercontent.com/97037684/149645773-302f4fbc-4476-4442-adac-650ea123f9de.mp4




https://user-images.githubusercontent.com/97037684/149645819-7f133fd0-6f8a-4d56-b0c1-1d29cea6a4c7.mp4



## Testes finais



https://user-images.githubusercontent.com/97037684/149646020-59e230fe-20b0-45bc-ad7a-104ca1fce77d.mp4




https://user-images.githubusercontent.com/97037684/149646030-e3ce5ff7-9e21-47c0-a643-dc62ff949062.mp4





https://user-images.githubusercontent.com/97037684/149646034-d2c8e083-a4e7-4813-9e0a-c57fa9880bb7.mp4







https://user-images.githubusercontent.com/97037684/149646050-ffcd8454-06f8-4bba-91da-be9efcde7dd1.mp4






## Agradecimentos:

Todos alunos e integrantes do grupo de estudo de Nova Lima "Desafio Maker- Carro Robô"!

Toda equipe da Secretaria de Educação de Nova Lima pelo Apoio e incentivo a educação Maker!

Ao Professor de (Pensamento Computacional e Robótica) orientador do grupo, Fernando Fernandes Dias Neves!

Ao idealizador do desafio Maker, Renato Aloi, pelo incentivo a educação tecnológica acessível a todos!  

E a todos que de uma forma ou outra, contribuíram com o nosso projeto e irá beneficiar com esse tutorial!

