/*


  Définition de l'IA du joueur artificiel de Rasende Roboter


*/


:- module( decision, [
	init/1,
	move/2
] ).

:- use_module( data ).

init(_).


/*
  move( +L, -ActionId )
*/


move( [TL, TR, BL, BR, T | _],[]) :- element(T, X, Y, TL, TR, BL, BR), writef('Coordonnees: %t,%t \n', [X, Y]), !.
/*
move( [_, _, _, _, T | _],[R, X]) :- T is 0, !, writeln('Il faut bouger nimporte quel robot'), write('\n'), random(1,4,X), random(1,4,R),  !.
move( [_, _, _, _, T | _],[0, X]) :- T > 0, T < 5, !, writeln('Il faut bouger le bleu'), write('\n'), random(1,4,X), !.
move( [_, _, _, _, T | _],[1, X]) :- T > 4, T < 9, !, writeln('Il faut bouger le vert'), write('\n'),random(1,4,X), !.
move( [_, _, _, _, T | _],[2, X]) :- T > 8, T < 13, !, writeln('Il faut bouger le jaune'), write('\n'), random(1,4,X), !.
move( [_, _, _, _, T | _],[3, X]) :- T > 12, T < 17, !, writeln('Il faut bouger le rouge'), write('\n'), random(1,4,X), !.
*/

/*
move( [_, _, _, _, _, BX, BY, GX, GY, YX, YY, RX, RY | _], [0,1]) :-
 writef("Blue:"), write(BX), writef(','), write(BY), write('\n'),
 writef("Green:"), write(GX), writef(','), write(GY), write('\n'),
 writef("Yellow:"), write(YX), writef(','), write(YY), write('\n'),
 writef("Red:"), write(RX), writef(','), write(RY), write('\n'),
 !.
*/

move( [0,0,0,0,  2, 6,1 | _], [0,1,0,4] ) :- !.
move( [0,0,0,0, 14, _,_, _,_, _,_, 5,15], [3,3,3,2,3,3,3,4] ) :- !.

move( _, [] ) :- !.
%        ^
%        |
%        Action: next configuration
