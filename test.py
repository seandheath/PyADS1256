import PyADS1256

ads = PyADS1256.ADS1256()

myid = ads.ReadID()

print(myid)
