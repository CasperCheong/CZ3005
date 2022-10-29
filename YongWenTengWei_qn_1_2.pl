company(sumsum) .
company(appy) .

business(galactica-s3) .

boss(stevey, appy) .
developed(sumsum, galactica-s3) .

stolen(stevey, galactica-s3) .

rival(sumsum, appy) .

unethical(X) :- boss(X, appy), developed(Z, Y), stolen(X, Y), rival(Z,appy), business(Y) .