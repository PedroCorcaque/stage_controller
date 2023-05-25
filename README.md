# Stage_Controller package

## Autores: Pedro Corçaque (135304), Emilly Lamotte (132821)

Esse pacote contem um arquivo launch que inicia o simulador 2D Stage e o nodo stage_controller para fazer o robô chegar no alvo desejado.
______

### Pré-requisitos

- [git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git)
- [ROS Noetic](http://wiki.ros.org/noetic/Installation)
______

### scripts/stage_controller.py

Foi implementado uma classe que, a partir da leitura do laser e da posição do robô no mundo, se movimenta em direção ao alvo solicitado, desviando de obstáculos se estiverem no seu caminho.
- Para a movimentação foi utilizado um Controle Proporcional, onde as constantes kp_linear e kp_angular podem ser alteradas no construtor da classe.
- Para o desvio de obstáculos foi implementado a estratégia de rotacionar ao redor do obstáculo até que seja possível seguir o caminho até o alvo.

Informações do robô:
- A variável robot_position é um array com 3 informações: posição em x, posição em y, e a orientação do robô em relação ao eixo x;
- A posição do robô é atualizada a partir da leitura do tópico /base_pose_ground_truth;
- A leitura do sensor é atualizada a partir da leitura do tópico /base_scan.
______

### Como rodar

1. Crie um workspace com o diretório src:

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
```
  
2. Clone o pacote dentro da pasta workspace/src

```
$ git clone https://github.com/PedroCorcaque/stage_controller.git
$ cd ~/catkin_ws
```

3. Compile o pacote

```
$ catkin_make
```

3.1. Se estiver usando bash

```
$ source devel/setup.bash
```

3.2. Se estiver usando zsh

```
$ source devel/setup.zsh
```

4. Inicie o launcher

```
$ roslaunch stage_controller launcher.launch
```
