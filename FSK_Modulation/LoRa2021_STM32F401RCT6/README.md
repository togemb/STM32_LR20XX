### Проект "Спутник-Королевец"

## App (бизнес-логика)
# Inc
*app.h* - 
# Src
*app.c* - использует только *board.h*

## Bsp (board support package)
# Inc
*apps_configuration.h* - параметры модуля
*board.h* - обертка аппаратной части
*lr2021.h* - 
# Src
*board.c* - 
*lr20xx_hal.c* - 
*lr2021.c*


Иерархия файлов

*.h/c*
     ^
     |
*.h/c*
     ^
     |
*.h/c*
     ^
     |
*lr2021.h/c*
     ^
     |
*lr20xx_hal.h/c*
     ^
     |
*board.h/c*