#Código do MOOS-IvP para pesquisa de mestrado

pContactSpawn -> Gera contatos que virão em direção do navio de maneira simulada quando determinadas variáveis forem escolhidas.

No caso ele vai gerar os contatos de forma a simular as regras do RIPEAM que queremos testar


pAstarColAvd -> Basicamente ele faz o collision avoidance desse contato utilizando o algoritmo A-star e também respeitando as regras do ripeam


Para gerar um contato -> uPokeDB SPAWN_CONTACT="heading=270,relative_bearing=0,distance=500,speed=5" (Rodar enquanto estiver rodando a missão)



Cronograma de testes:

- Primeiro rodar com o algoritmo do próprio MOOS-IvP seguindo sempre a mesma derrota com os mesmos pontos e navios vindo em situações diferentes (marcar pontos para que apareçam os contatos)
(Tentar simular antes para ver se está tudo okay)

- Depois rodar com o pColAvd que é o A* com as mesmas condições

- Depois rodar com o Velocity Obstacle com as mesmas condições



