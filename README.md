# Projeto 1

Da disciplina de Robótica Computacional do Insper.

O propósito do projeto era configurar um robô (Neato), para que este tivesse as seguinte funções:
  - Reconhecer 3 marcadores do tipo Marker e em cada caso fazer um ação diferente, sendo uma delas seguir;
  - Utilizar um método de sobrevivencia sendo ele os principais: bump ou sensor do laser.
  
  Nosso robo, ao reconhecer o Marker 50, segue-o até 70 cm de distância. Já o marker 100, ao reconhece-lo, se aproxima, também, até 70cm e então gira 180 graus se afastando. E por fim, no caso do marker 150, o robô também se aproxima e ao chegar a 70cm, para todo o funcionamento do robô: movimento, bump e laser.
  O bump reconhece se a batido foi na frente, na esquerda ou direita e gira para o lado oposto. Já o laser reconhece os objetos na proximidade(no máximo 50cm) e se afasta.
  Além disso, o robô também diminui a velocidade ao se aproximar de um marcador.


Bruna Kimura implementou a função recebe_laser e suas respectivas configurações e variáveis e também a função de diminuir a velocidade conforme a distância do marcador..
Vitória Camilo implementou o tempo utilizado na busca do marcador e os comentários que acompanham o código.
A função recebe_bump e suas configurações, e a função recebe_marcador, o mecanismo de busca do marcador e configuração dos ids foram implementadas em conjunto.
No código tivemos também ajuda dos professores Fábio Miranda e Igor Montagner.
