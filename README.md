# GCP_Finder 
O GCP Finder é um programa identifica Aruco Markers em imagens.

Este programa surgiu no âmbito de uma dissertação de Mestrado do Departamento de Informática da Faculdade de Ciência e Tecnologia da Universidade NOVA de Lisboa em colaboração com o Laboratório Nacional de Engenharia Civil, com o intuito de tornar o levantamento topográfico de estruturas de engenharia, nomeadamente barragens, mais prático, preciso e rápido.

O programa começa por separar as imagens que contêm um ponto de controlo daquelas que nâo contêm um ponto de controlo. Após essa operação, o programa 
identifica o centro dos pontos de controlo nas imagens que foram selecionadas e gera um ficheiro gcp_list.txt que irá conter a localização dos pontos de controlo em pixeis para cada imagem, já com o formato correto para ser utilizado como ficheiro de configuração no Open Drone Map.

Instruções de utilização:

$ git clone https://github.com/octaviojardim/GCP_Finder

$ cd GCP_Finder/

$ make install

$ source ./venv/bin/activate

$ python GCP_Finder.py arg1 arg2 arg3 [options] opt1

• arg1: Caminho para a pasta com as imagens a processar.

• arg2: Caminho para o ficheiro de texto com a localização GPS dos pontos de controlo
colocados no terreno.

• arg3: Percentagem de borda a retirar de cada imagem.

• opt1: Caso o utilizador queira guardar as imagens em que foram encontrados pontos
de controlo, este deve indicar o caminho da pasta para onde estas imagens devem
ser guardadas.

O resultado final ("gcp_list.txt") é guardado na diretoria corrente. Assim, um exemplo
de comando a executar seria:

$ python GCP_Finder.py ./fotos/ gps_list.txt 20 ./fotos_selecionadas/
