/* predicates */
company(sumsum) .
company(appy) .
competitor(sumsum,appy) .
smartphoneTechnology(galactica-s3) .
developed(sumsum,galactica-s3) .
boss(stevey, appy) .
stolen(stevey, galactica-s3) .

/* rules */
techDevelopedBy(X,Y) :- business(X), company(Y) .
business(X) :- smartphoneTechnology(X) .
competitor(X, Y) :- competitor(Y, X) .
rival(X, Y) :- competitor(X, Y) .
unethical(X) :- boss(X, Y), stolen(X, Z), developed(W,Z), techDevelopedBy(Z,W), rival(Y, W) .