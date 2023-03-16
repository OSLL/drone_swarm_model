from PyQt5 import QtWidgets, Qt
import sys
    
class Window(Qt.QWidget):
    def __init__(self):
        super().__init__()

        layout = Qt.QVBoxLayout(self)
        self.setStyleSheet("QWidget {background-color:rgb(37,37,38);color:rgb(255,255,255)}"
                           "QLineEdit {background-color:rgb(51,51,55);}"
                           "QTextEdit {background-color:rgb(51,51,55);}"
                           "QPushButton{background-color:rgb(51,51,55)}"
                           "QPushButton:hover{background-color:rgb(61,61,65)}")

        #для будущего вывода ответов от XQueue
        self.response_text = Qt.QLabel("")

        #ввод task_id
        self.task_id = QtWidgets.QLineEdit()
        
        #ввод решения
        self.solution = QtWidgets.QTextEdit()

        #добавление на layout
        layout.addWidget(Qt.QLabel("Введите номер задания:"))
        layout.addWidget(self.task_id)
        layout.addWidget(Qt.QLabel("Введите ваше решение:"))
        layout.addWidget(self.solution)
        
        self.btn = Qt.QPushButton("Отправить решение")
        self.btn.clicked.connect(self.send_solution)
        
        layout.addWidget(self.btn)
        layout.addWidget(self.response_text)

    def send_solution(self):
        """
        временно - вывод введенного
        ----
        реализовать отправку решения в XQueue
        и, возможно, валидацию
        """
        task_id = self.task_id.text()
        solution = self.solution.toPlainText()
        print(task_id, solution)
        pass
    
if __name__ == "__main__":
    app = Qt.QApplication([])
    w = Window()
    w.show()
    app.exec()
