/* predicates */
company(sumsum) .
company(appy) .
competitor(sumsum,appy) .
smartphonetechnology(galactica-s3) .
boss(stevey, appy) .
developed(sumsum, galactica-s3) .
stolen(stevey, galactica-s3) .

/* rules */
business(X) :- smartphonetechnology(X) .
competitor(X, Y) :- competitor(Y, X) .
rival(X, Y) :- competitor(X, Y) .
unethical(X) :- boss(X, Y), stolen(X, Z), developed(W, Z), business(Z),company(W), rival(Y, W) .