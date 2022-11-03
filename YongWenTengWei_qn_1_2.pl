/* predicates */
company(sumsum) .
company(appy) .
competitor(sumsum,appy) .
smartphonetechnology(galactica-s3) .
boss(stevey, appy) .
stolen(stevey, galactica-s3) .

/* rules */
developed(X,Y) :- company(X), smartphonetechnology(Y) .
business(X) :- smartphonetechnology(X) .
competitor(X, Y) :- competitor(Y, X) .
rival(X, Y) :- competitor(X, Y) .
unethical(X) :- boss(X, Y), stolen(X, Z), developed(W, Z), rival(Y, W) .