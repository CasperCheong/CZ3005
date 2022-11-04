/* predicates */
company(sumsum) .
company(appy) .
competitor(sumsum,appy) .
smart_phone_technology(galactica-s3) .
developed(sumsum,galactica-s3) .
boss(stevey, appy) .
stolen(stevey, galactica-s3) .

/* rules */
tech_developed_by(X,Y) :- developed(Y,X), business(X), company(Y) .
business(X) :- smart_phone_technology(X) .
competitor(X, Y) :- competitor(Y, X) .
rival(X, Y) :- competitor(X, Y) .
unethical(X) :- boss(X, Y), stolen(X, Z), tech_developed_by(Z,W), rival(Y, W) .