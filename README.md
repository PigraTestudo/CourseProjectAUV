## CourseProjectAUV
Код для курсовой работы 6-го семестра. Задача состояла в программировании АНПА из V-rep для поиска красных кубиков.
**Куча минусов:**
- Интерфейса нет
- Навигация реализована странно
	- Непонятные условия-if'ы для сил
	- Проблема арктангенса с углами в 90*
	- Зачем-то контролируется крен, хотя не нужно
	- Нет остойчивости в скрипте питона
- Камера плохо засчитывает объекты (наверное, правится изменением диапазона размеров распознаваемых объектов)

_AUV_model.ttt_ - простой и маленький полигон.
<br> _Poligon_ORL.ttt_ - большой и сложный, на котором курсовой сдавался. 
<br> На маленьком аппарат работает сносно, на большом - не находит кубы и не до конца стабильно себя ведет. 
