DS4_NAMES = [
        ("ds4white", "90:FB:A6:D1:C0:CB"),
        ("ds4blue", "30:0E:D5:94:FF:FB"),
        ("ds4black", "D0:27:88:59:DF:A1"),
        ("ds4red", "90:FB:A6:92:BB:F3"),
        ("ds4berry", "8C:41:F2:8C:DD:A2"),
        ("ds4navy", "8C:41:F2:E1:76:BE"),
        ("ds4orange", "8C:41:F2:16:A8:BA"),
        ("ds4camo", "A0:AB:51:B3:78:A8"),
        ("ds4stripe", "4C:B9:9B:0B:24:56"),
        ]

DS4_NAME_TO_MAC = { name: mac for (name, mac) in DS4_NAMES }
DS4_MAC_TO_NAME = { mac: name for (name, mac) in DS4_NAMES }
