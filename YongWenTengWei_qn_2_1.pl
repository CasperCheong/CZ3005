male(charles) .
male(andrew) . 
male(edward) .

female(elizabeth) .
female(ann) .

parent(elizabeth, charles) .
parent(elizabeth, andrew) .
parent(elizabeth, edward) .
parent(elizabeth, ann) .

son(charles, elizabeth) .
son(andrew, elizabeth) .
son(edward, elizabeth) .

daughter(ann, elizabeth) .



successor(X, Y) :- son(Y,X), parent(X, Y) .
successor(X, Y) :- daughter(Y,X), parent(X, Y) . 