class WriteCoil:
    writecoil_start = 0x00
    writecoil_end = 0x1F
    writecoil_names = {
        "AMR_VALID" : 0x1000,
        "LOAD_REQUEST" : 0x1010,
        "UNLOAD_REQUEST" : 0x1011,
        "MOVE_REQUEST" : 0x1012,
        "CS" : 0x1015,
    }
    writecoil_sizes = {
        "AMR_VALID" : 1, 
        "LOAD_REQUEST" : 1, 
        "UNLOAD_REQUEST" : 1, 
        "MOVE_REQUEST" : 1, 
        "CS" : 1, 
    }
    def __init__(self):
        self.memory = {addr: 0 for addr in range(self.writecoil_start, self.writecoil_end + 1)}

    def set_value_by_name(self, name, value):
        if name not in self.writecoil_names:
            raise ValueError(f"Address name '{name}' not found.")
        addr = self.writecoil_names[name]
        # size = self.address_sizes.get(name, 1)  # 기본 크기는 1
        size = self.writecoil_sizes[name]
        if size == 1:
            self.memory[addr] = value
        else:
            for i in range(size):
                self.memory[addr + i] = (value >> (16 * i)) & 0xFFFF

    def get_value_by_name(self, name):
        if name not in self.writecoil_names:
            raise ValueError(f"Address name '{name}' not found.")
        addr = self.writecoil_names[name]
        # size = self.address_sizes.get(name, 1)  # 기본 크기는 1
        size = self.writecoil_sizes[name]
        if size == 1:
            return self.memory[addr]
        else:
            value = 0
            for i in range(size):
                value |= self.memory[addr + i] << (16 * i)
            return value

    def serialize(self):
        serialized_data = [self.memory[addr] for addr in range(self.writecoil_start, self.writecoil_end + 1)]
        return serialized_data

    def deserialize(self, data):
        for addr, value in zip(range(self.writecoil_start, self.writecoil_end + 1), data):
            self.memory[addr] = value