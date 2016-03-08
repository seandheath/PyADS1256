import pyads1256

ads = pyads1256.ADS1256()

myid = ads.ReadID()

print(myid)
