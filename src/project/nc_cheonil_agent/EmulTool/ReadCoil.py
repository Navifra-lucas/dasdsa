class ReadCoil:
    readcoil_start = 0x1000
    readcoil_end = 0x101F
    readcoil_names = {
        "READY" : 0x1000,
        "BUSY" : 0x1001,
        "AUTO" : 0x1002,
        "MANUAL" : 0x1003,
        "ALARM" : 0x1004,
        "HOME" : 0x1005,
        "TR_REQUEST" : 0x1010,
        "COMPT" : 0x1011,
    }
    readcoil_sizes = {
        "READY" : 1, 
        "BUSY" : 1, 
        "AUTO" : 1, 
        "MANUAL" : 1, 
        "ALARM" : 1, 
        "HOME" : 1, 
        "TR_REQUEST" : 1, 
        "COMPT" : 1, 
    }

    def __init__(self):
        self.memory = {addr: 0 for addr in range(self.readcoil_start, self.readcoil_end + 1)}

    def set_value_by_name(self, name, value):
        if name not in self.readcoil_names:
            raise ValueError(f"Address name '{name}' not found.")
        addr = self.readcoil_names[name]
        # size = self.address_sizes.get(name, 1)  # 기본 크기는 1
        size = self.readcoil_sizes[name]
        if size == 1:
            self.memory[addr] = value
        else:
            for i in range(size):
                self.memory[addr + i] = (value >> (16 * i)) & 0xFFFF

    def get_value_by_name(self, name):
        if name not in self.readcoil_names:
            raise ValueError(f"Address name '{name}' not found.")
        addr = self.readcoil_names[name]
        # size = self.address_sizes.get(name, 1)  # 기본 크기는 1
        size = self.readcoil_sizes[name]
        if size == 1:
            return self.memory[addr]
        else:
            value = 0
            for i in range(size):
                value |= self.memory[addr + i] << (16 * i)
            return value

    def serialize(self):
        serialized_data = [self.memory[addr] for addr in range(self.readcoil_start, self.readcoil_end + 1)]
        return serialized_data

    def deserialize(self, data):
        for addr, value in zip(range(self.readcoil_start, self.readcoil_end + 1), data):
            self.memory[addr] = value