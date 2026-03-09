import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QLabel, QPushButton, QTabWidget, QHBoxLayout, QCheckBox, QTableWidget, QTableWidgetItem, QComboBox, QLineEdit, QSpacerItem, QSizePolicy, QMessageBox
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from ReadRegister import ReadRegister
from WriteRegister import WriteRegister
from ReadCoil import ReadCoil
from WriteCoil import WriteCoil
from pymodbus.client import ModbusTcpClient
from enum import Enum
import debugpy

class EmulatorThread(QThread):
    update_signal = pyqtSignal()

    def __init__(self, readRegister, writeRegister, readCoil, writeCoil, parent=None):
        super().__init__(parent)
        self.readRegister = readRegister
        self.writeRegister = writeRegister
        self.readCoil = readCoil
        self.writeCoil = writeCoil
        self.client = ModbusTcpClient('localhost')
        self.running = False
        
        readregister_result = self.client.read_holding_registers(self.readRegister.readregister_start, len(self.readRegister.serialize()))
        if not readregister_result.isError():
            self.readRegister.deserialize(readregister_result.registers)

        readcoil_result = self.client.read_coils(self.readCoil.readcoil_start, len(self.readCoil.serialize()))
        if not readcoil_result.isError():
            self.readRegister.deserialize(readcoil_result.bits)

################################################################################3
    def run(self):
        debugpy.debug_this_thread()  # 현재 스레드를 디버그 대상으로 설정
        while self.running:
            if self.client.connect():
                # Read from holding register starting at 300
                writeregister_result = self.client.read_holding_registers(self.writeRegister.writeregister_start, len(self.writeRegister.serialize()))
                if not writeregister_result.isError():
                    self.writeRegister.deserialize(writeregister_result.registers)
                    print(f'Read Register Data: {writeregister_result.registers}')

                writecoil_result = self.client.read_coils(self.writeCoil.writecoil_start, len(self.writeCoil.serialize()))
                if not writecoil_result.isError():
                    self.writeCoil.deserialize(writecoil_result.bits)
                    print(f'Read Clil Data: {writecoil_result.bits}')

                # Write to holding register starting at 400
                if self.readRegister.get_value_by_name("HEARTBEAT") >= 0xFF :
                    self.readRegister.set_value_by_name("HEARTBEAT", 0) 
                else :
                    self.readRegister.set_value_by_name("HEARTBEAT",self.readRegister.get_value_by_name("HEARTBEAT")  + 1)

                # write_registers_data = self.readRegister.serialize()
                # self.client.write_registers(self.readRegister.readregister_start, write_registers_data)
                # print(f'Write register Data: {write_registers_data}')

                # write_coil_data = self.readCoil.serialize()
                # self.client.write_registers(self.readCoil.readcoil_start, write_coil_data)
                # print(f'Write coil Data: {write_coil_data}')

                writeregister_result = self.client.read_holding_registers(self.readRegister.readregister_start, len(self.readRegister.serialize()))
                if not writeregister_result.isError():
                    self.readRegister.deserialize(writeregister_result.registers)
                    print(f'Write Register Data: {writeregister_result.registers}')

                writecoil_result = self.client.read_coils(self.readCoil.readcoil_start, len(self.readCoil.serialize()))
                if not writecoil_result.isError():
                    self.readCoil.deserialize(writecoil_result.bits)
                    print(f'Write Clil Data: {writecoil_result.bits}')

                self.client.close()
                self.update_signal.emit()
            self.msleep(500)

    def stop(self):
        self.running = False
        self.wait()
            
class RussellEmulTool(QMainWindow):
    def __init__(self):
        super().__init__()

        self.initUI()
        self.readRegister = ReadRegister()
        self.writeRegister = WriteRegister()
        self.readCoil = ReadCoil()
        self.writeCoil = WriteCoil()
        self.emulatorThread = EmulatorThread(self.readRegister, self.writeRegister, self.readCoil, self.writeCoil)
        self.emulatorThread.update_signal.connect(self.updateUI)

    def initUI(self):
        self.setWindowTitle('Russell Emul Tool')
        self.setGeometry(100, 100, 800, 600)  # 윈도우 위치 (100, 100)과 크기 (800, 600) 설정

        self.tabs = QTabWidget()
        self.statusTab = QWidget()
        self.registerTab = QWidget()
        self.coilTab = QWidget()
        
        self.tabs.addTab(self.statusTab, "Status")
        self.tabs.addTab(self.registerTab, "Register")
        self.tabs.addTab(self.coilTab, "Coil")
        
        self.initStatusTab()
        self.initRegisterTab()
        self.initCoilTab()

        layout = QVBoxLayout()
        layout.addWidget(self.tabs)

        container = QWidget()
        container.setLayout(layout)

        self.setCentralWidget(container)

    def initStatusTab(self):
        layout = QVBoxLayout()

        SpaceLayout = QHBoxLayout()
        self.SpaceLabel = QLabel('', self)
        SpaceLayout.addWidget(self.SpaceLabel)

        # Connection Status Label
        self.connectionStatusLabel = QLabel('Not Connected', self)
        self.connectionStatusLabel.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.connectionStatusLabel)

        # Connect/Disconnect Button
        self.toggleConnectionButton = QPushButton('Connect', self)
        self.toggleConnectionButton.clicked.connect(self.toggleConnection)
        layout.addWidget(self.toggleConnectionButton)
        

        # Create a spacer item to push the components to the top
        spacer = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)
        layout.addItem(spacer)

        self.statusTab.setLayout(layout)
        self.statusTab.setLayout(SpaceLayout)

    def initRegisterTab(self):
        layout = QVBoxLayout()

        self.dataFormatCombo = QComboBox(self)
        self.dataFormatCombo.addItems(['Decimal', 'Binary'])
        self.dataFormatCombo.currentIndexChanged.connect(self.updateUI)
        layout.addWidget(self.dataFormatCombo)

        self.readRegisterDataTable = QTableWidget(self)
        self.readRegisterDataTable.setColumnCount(2)
        self.readRegisterDataTable.setHorizontalHeaderLabels(['Address', 'Value'])
        layout.addWidget(self.readRegisterDataTable)

        self.writeRegisterDataTable = QTableWidget(self)
        self.writeRegisterDataTable.setColumnCount(2)
        self.writeRegisterDataTable.setHorizontalHeaderLabels(['Address', 'Value'])
        layout.addWidget(self.writeRegisterDataTable)

        self.registerTab.setLayout(layout)

    def initCoilTab(self):
        layout = QVBoxLayout()

        # self.dataFormatCombo = QComboBox(self)
        # self.dataFormatCombo.addItems(['Decimal', 'Binary'])
        # self.dataFormatCombo.currentIndexChanged.connect(self.updateUI)
        # layout.addWidget(self.dataFormatCombo)

        self.readCoilDataTable = QTableWidget(self)
        self.readCoilDataTable.setColumnCount(2)
        self.readCoilDataTable.setHorizontalHeaderLabels(['Address', 'Value'])
        layout.addWidget(self.readCoilDataTable)

        self.writeCoilDataTable = QTableWidget(self)
        self.writeCoilDataTable.setColumnCount(2)
        self.writeCoilDataTable.setHorizontalHeaderLabels(['Address', 'Value'])
        layout.addWidget(self.writeCoilDataTable)
        
        self.coilTab.setLayout(layout)

    def toggleConnection(self):
        if not self.emulatorThread.running:
            self.emulatorThread.running = True
            self.emulatorThread.start()
            self.connectionStatusLabel.setText('Connected')
            self.toggleConnectionButton.setText('Disconnect')
        else:
            self.emulatorThread.stop()
            self.connectionStatusLabel.setText('Not Connected')
            self.toggleConnectionButton.setText('Connect')
            
    def showMessage(self, title, message):
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Information)
        msg.setWindowTitle(title)
        msg.setText(message)
        msg.setStandardButtons(QMessageBox.Ok)
        msg.exec_()
        
    def updateUI(self):
        print("Updating UI...")
        # 필요한 경우 UI 업데이트 로직 추가
        
        format = self.dataFormatCombo.currentText()
        self.updateTable(self.readRegisterDataTable, self.readRegister.serialize(), format)
        self.updateTable(self.writeRegisterDataTable, self.writeRegister.serialize(), format)
        self.updateTable(self.readCoilDataTable, self.readCoil.serialize(), 'Binary')
        self.updateTable(self.writeCoilDataTable, self.writeCoil.serialize(), 'Binary')

    def updateTable(self, table, data, format):
        table.setRowCount(len(data))
        for i, value in enumerate(data):
            table.setItem(i, 0, QTableWidgetItem(str(hex(i))))
            if format == 'Binary':
                table.setItem(i, 1, QTableWidgetItem(str(bool(value))))
            else:
                table.setItem(i, 1, QTableWidgetItem(str(int(value))))

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = RussellEmulTool()
    ex.show()
    sys.exit(app.exec_())
