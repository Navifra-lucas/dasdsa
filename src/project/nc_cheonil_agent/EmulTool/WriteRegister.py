class WriteRegister:
    writeregister_start = 0x00
    writeregister_end = 0x08
    writeregister_names = {
        "HEARTBEAT" : 0x1000,
        "TARGET_LEVEL" : 0x1001,
    }
    writeregister_sizes = {
        "HEARTBEAT" : 1, 
        "TARGET_LEVEL" : 1, 
    }
    def __init__(self):
        self.memory = {addr: 0 for addr in range(self.writeregister_start, self.writeregister_end + 1)}

    def set_value_by_name(self, name, value):
        if name not in self.writeregister_names:
            raise ValueError(f"Address name '{name}' not found.")
        addr = self.writeregister_names[name]
        # size = self.address_sizes.get(name, 1)  # 기본 크기는 1
        size = self.writeregister_sizes[name]
        if size == 1:
            self.memory[addr] = value
        else:
            for i in range(size):
                self.memory[addr + i] = (value >> (16 * i)) & 0xFFFF

    def get_value_by_name(self, name):
        if name not in self.writeregister_names:
            raise ValueError(f"Address name '{name}' not found.")
        addr = self.writeregister_names[name]
        # size = self.address_sizes.get(name, 1)  # 기본 크기는 1
        size = self.writeregister_sizes[name]
        if size == 1:
            return self.memory[addr]
        else:
            value = 0
            for i in range(size):
                value |= self.memory[addr + i] << (16 * i)
            return value

    def serialize(self):
        serialized_data = [self.memory[addr] for addr in range(self.writeregister_start, self.writeregister_end + 1)]
        return serialized_data

    def deserialize(self, data):
        for addr, value in zip(range(self.writeregister_start, self.writeregister_end + 1), data):
            self.memory[addr] = value
