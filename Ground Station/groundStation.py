import groundStationClass


if __name__ == "__main__":
    gs = groundStationClass.groundStation()
    if not gs.connect():
        print("[main] Could not connect to RF receiver")
        
        