# Seguidor de linha utilizando ROS e OpenCV

Este script em Python utiliza o ROS (Robot Operating System) e a biblioteca OpenCV para controlar um drone seguindo uma linha. Ele se baseia em detecção de cor para identificar a linha a ser seguida na imagem capturada pela câmera do drone.

## Dependências e configurações

### Bibliotecas necessárias:
- rospy: Biblioteca ROS para Python
- cv2: OpenCV para manipulação de imagens
- numpy: Manipulação de arrays multidimensionais
- math: Funções matemáticas

### Mensagens ROS utilizadas:
- `geometry_msgs.msg.Vector3`: Mensagem para representar vetores tridimensionais.
- `geometry_msgs.msg.Point`: Mensagem para representar um ponto tridimensional.
- `mavros_msgs.msg.PositionTarget`: Mensagem para definir a posição e a orientação do drone.
- `tf2_msgs.msg.TFMessage`: Mensagem para representar transformações entre coordenadas.

### Variáveis de controle:
- `IMSHOW`: Ativa/desativa a exibição das imagens durante a execução.
- `RECORD`: Ativa/desativa a gravação de vídeo durante a execução.
- `TESTNUM`: Número do teste para a gravação de vídeo. Irá aparecer no nome do arquivo de vídeo.
- `ONLY_YAW`: O drone irá ajustar apenas sua orientação de acordo com a linha, não se movimentando linearmente.

### Parâmetros de controle PID:
- `Kp`, `Ki`, `Kd`: Parâmetros do controlador PID para correção do erro lateral.
- `Kp_ang`, `Ki_ang`, `Kd_ang`: Parâmetros do controlador PID para correção do ângulo do drone.


## Funcionamento do Código

O código consiste principalmente na classe `line_follower`, que possui métodos para detectar a linha na imagem, calcular os ajustes necessários para manter o drone alinhado com a linha e publicar os comandos de controle.

Uma zoom in é aplicado à imagem coletada para aumentar a imagem da linha na tela, caso ela seja muito fina. É possível intensificar ou diminuir o zoom com o parâmetro `scale` passado na função `zoom()`.

O método `line_detect` processa a imagem para detectar a linha utilizando uma máscara na faixa de cor desejada (mais sobre máscara de cores em https://github.com/SkyRats/calibrador-cores). Em seguida, calcula os erros lateral e angular para corrigir a trajetória do drone. Os erros são estimados a partir da distância em pixel entre o centro do retângulo correspondente à linha na imagem e o ponto central da imagem gerada pela câmera. O ângulo se refere a diferença angular entre o eixo vertical do retângulo que representa a linha e o eixo vertical central da imagem.

O método `detection_loop` captura continuamente imagens da câmera do drone, processa-as para detecção da linha e aplica os ajustes de controle necessários.

## Observações

### Velocidade

A velocidade pode ser alterada utilizando a variável `self.velocity` ou, diretamente pela qground, mudando o parâmetro `WPNAV_SPEED`.

### Orientação da câmera e imagem

Certifique-se que a orientação da câmera está de acordo com "a frente" do drone. Utilize o imshow para checar que a linha aparece verticalmente na imagem.

Em seguida, utilizando somendo o yaw, verifique que o drone se alinha verticalmente com a linha. Caso ele se alinhe horizontalmente com ela, comente a seguinte linha:

```
# Rotate image
        angle += 90
```
