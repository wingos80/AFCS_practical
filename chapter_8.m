   c   l   e   a   r      
      
   g   l   o   b   a   l       f   i   _   f   l   a   g   _   S   i   m   u   l   i   n   k      
      
   %   %       T   r   i   m       a   i   r   c   r   a   f   t       t   o       d   e   s   i   r   e   d       a   l   t   i   t   u   d   e       a   n   d       v   e   l   o   c   i   t   y      
   %   %      
   %       l   a   s   t   n   a   m   e       B   .   ,       l   a   s   t       n   u   m   b   e   r       3   0   0      
   a   l   t   i   t   u   d   e       =       5   0   0   0   ;      
   v   e   l   o   c   i   t   y       =       3   0   0   ;      
      
      
   d   i   s   p   (   '   A   t       w   h   a   t       f   l   i   g   h   t       c   o   n   d   i   t   i   o   n       w   o   u   l   d       y   o   u       l   i   k   e       t   o       t   r   i   m       t   h   e       F   -   1   6   ?   '   )   ;      
   d   i   s   p   (   '   1   .           S   t   e   a   d   y       W   i   n   g   s   -   L   e   v   e   l       F   l   i   g   h   t   .   '   )   ;      
   d   i   s   p   (   '   2   .           S   t   e   a   d   y       T   u   r   n   i   n   g       F   l   i   g   h   t   .   '   )   ;      
   d   i   s   p   (   '   3   .           S   t   e   a   d   y       P   u   l   l   -   U   p       F   l   i   g   h   t   .   '   )   ;      
   d   i   s   p   (   '   4   .           S   t   e   a   d   y       R   o   l   l       F   l   i   g   h   t   .   '   )   ;      
   F   C   _   f   l   a   g       =       1   ;      
      
   d   i   s   p   (   '   W   e       a   r   e       w   o   r   k   i   n   g       w   i   t   h   :   '   )   ;      
   d   i   s   p   (   F   C   _   f   l   a   g   )      
      
   x   _   a       =       5   .   9   ;      
   g   _   d       =       3   2   .   1   7   ;       %   g   r   a   v   i   t   a   t   i   o   n   a   l       a   c   c   e   l   e   r   a   t   i   o   n       i   n       f   t       p   e   r       s   e   c   o   n   d       s   q   u   a   r   e   d      
      
   %   %       I   n   i   t   i   a   l       g   u   e   s   s       f   o   r       t   r   i   m      
   %   %      
   t   h   r   u   s   t       =       5   0   0   0   ;                                           %       t   h   r   u   s   t   ,       l   b   s      
   e   l   e   v   a   t   o   r       =       -   0   .   0   9   ;                               %       e   l   e   v   a   t   o   r   ,       d   e   g   r   e   e   s      
   a   l   p   h   a       =       8   .   4   9   ;                                                           %       A   O   A   ,       d   e   g   r   e   e   s      
   r   u   d   d   e   r       =       -   0   .   0   1   ;                                                       %       r   u   d   d   e   r       a   n   g   l   e   ,       d   e   g   r   e   e   s      
   a   i   l   e   r   o   n       =       0   .   0   1   ;                                                   %       a   i   l   e   r   o   n   ,       d   e   g   r   e   e   s      
      
   %   %       F   i   n   d       t   r   i   m       f   o   r       l   o   f   i       m   o   d   e   l       a   t       d   e   s   i   r   e   d       a   l   t   i   t   u   d   e       a   n   d       v   e   l   o   c   i   t   y      
   %   %      
   d   i   s   p   (   '   T   r   i   m   m   i   n   g       L   o   w       F   i   d   e   l   i   t   y       M   o   d   e   l   :   '   )   ;      
   f   i   _   f   l   a   g   _   S   i   m   u   l   i   n   k       =       0   ;      
   [   t   r   i   m   _   s   t   a   t   e   _   l   o   ,       t   r   i   m   _   t   h   r   u   s   t   _   l   o   ,       t   r   i   m   _   c   o   n   t   r   o   l   _   l   o   ,       d   L   E   F   ,       x   u   _   l   o   ]       =       t   r   i   m   _   F   1   6   (   t   h   r   u   s   t   ,       e   l   e   v   a   t   o   r   ,       a   l   p   h   a   ,       a   i   l   e   r   o   n   ,       r   u   d   d   e   r   ,       v   e   l   o   c   i   t   y   ,       a   l   t   i   t   u   d   e   ,       F   C   _   f   l   a   g   )   ;      
      
   %   %       F   i   n   d       t   h   e       s   t   a   t   e       s   p   a   c   e       m   o   d   e   l       f   o   r       t   h   e       l   o   f   i       m   o   d   e   l       a   t       t   h   e       d   e   s   i   r   e   d       a   l   t       a   n   d       v   e   l   .      
   %   %      
   t   r   i   m   _   s   t   a   t   e   _   l   i   n       =       t   r   i   m   _   s   t   a   t   e   _   l   o   ;       t   r   i   m   _   t   h   r   u   s   t   _   l   i   n       =       t   r   i   m   _   t   h   r   u   s   t   _   l   o   ;       t   r   i   m   _   c   o   n   t   r   o   l   _   l   i   n       =       t   r   i   m   _   c   o   n   t   r   o   l   _   l   o   ;      
   o   p   e   r   a   t   i   n   g   _   p   o   i   n   t       =       o   p   e   r   p   o   i   n   t   (   '   L   I   N   _   F   1   6   B   l   o   c   k   '   )   ;       %       r   e   t   r   i   e   v   e   s       i   n   i   t   i   a   l       c   o   n   d   i   t   i   o   n   s       f   r   o   m       i   n   t   e   g   r   a   t   o   r   s      
   o   p   e   r   a   t   i   n   g   _   p   o   i   n   t   .   I   n   p   u   t   s   (   1   )   .   u       =       t   r   i   m   _   t   h   r   u   s   t   _   l   i   n   ;       o   p   e   r   a   t   i   n   g   _   p   o   i   n   t   .   I   n   p   u   t   s   (   2   )   .   u       =       t   r   i   m   _   c   o   n   t   r   o   l   _   l   i   n   (   1   )   ;      
   o   p   e   r   a   t   i   n   g   _   p   o   i   n   t   .   I   n   p   u   t   s   (   3   )   .   u       =       t   r   i   m   _   c   o   n   t   r   o   l   _   l   i   n   (   2   )   ;       o   p   e   r   a   t   i   n   g   _   p   o   i   n   t   .   I   n   p   u   t   s   (   4   )   .   u       =       t   r   i   m   _   c   o   n   t   r   o   l   _   l   i   n   (   3   )   ;      
      
   S   S   _   l   o       =       l   i   n   e   a   r   i   z   e   (   '   a   c   c   e   l   e   r   o   m   e   t   e   r   _   L   I   N   _   F   1   6   B   l   o   c   k   '   )   ;      
      
   %   %       G   l   i   d   e   S   l   o   p   e       c   a   l   c   u   l   a   t   o   r      
   g   l   i   d   e   _   s   t   a   t   e   s       =       [   3   ,       7   ,       8   ,       5   ,       1   1   ]   ;                       %   h   ,       V   t   ,       a   l   p   h   a   ,       t   h   e   t   a   ,       q      
   g   l   i   d   e   _   i   n   p   u   t   s       =       [   1   3   ,       1   4   ]   ;                                                       %   t   h   r   u   s   t   ,       e   l   e   v   a   t   o   r      
   S   S   _   g   l   i   d   e   _   A       =       S   S   _   l   o   .   A   (   g   l   i   d   e   _   s   t   a   t   e   s   ,       g   l   i   d   e   _   s   t   a   t   e   s   )   ;      
   S   S   _   g   l   i   d   e   _   B       =       S   S   _   l   o   .   A   (   g   l   i   d   e   _   s   t   a   t   e   s   ,       g   l   i   d   e   _   i   n   p   u   t   s   )   ;      
   S   S   _   g   l   i   d   e   _   C       =       S   S   _   l   o   .   C   (   g   l   i   d   e   _   s   t   a   t   e   s   ,       g   l   i   d   e   _   s   t   a   t   e   s   )   ;      
   S   S   _   g   l   i   d   e   _   D       =       S   S   _   l   o   .   C   (   g   l   i   d   e   _   s   t   a   t   e   s   ,       g   l   i   d   e   _   i   n   p   u   t   s   )   ;      
      
   s   s   _   s   y   s       =       s   s   (   S   S   _   g   l   i   d   e   _   A   ,       S   S   _   g   l   i   d   e   _   B   ,       S   S   _   g   l   i   d   e   _   C   ,       S   S   _   g   l   i   d   e   _   D   )   ;      
   t   f   _   s   y   s       =       t   f   (   s   s   _   s   y   s   )   ;      
   t   h   r   u   s   t   _   2   _   o   u   t   s       =       m   i   n   r   e   a   l   (   t   f   _   s   y   s   (   :   ,   1   )   )   ;       t   h   r   u   s   t   _   2   _   o   u   t   s   .   I   n   p   u   t   N   a   m   e       =       '   d   _   t   h   r   u   s   t   '   ;       t   h   r   u   s   t   _   2   _   o   u   t   s   .   O   u   t   p   u   t   N   a   m   e       =       {   '   h   '   ,       '   V   t   '   ,       '   a   l   p   h   a   '   ,       '   t   h   e   t   a   '   ,       '   q   '   }   ;      
   e   l   e   v   a   t   o   r   _   2   _   o   u   t   s       =       m   i   n   r   e   a   l   (   t   f   _   s   y   s   (   :   ,   2   )   )   ;       e   l   e   v   a   t   o   r   _   2   _   o   u   t   s   .   I   n   p   u   t   N   a   m   e       =       '   d   _   e   l   e   v   a   o   t   r   '       ;       e   l   e   v   a   t   o   r   _   2   _   o   u   t   s   .   O   u   t   p   u   t   N   a   m   e       =       {   '   h   '   ,       '   V   t   '   ,       '   a   l   p   h   a   '   ,       '   t   h   e   t   a   '   ,       '   q   '   }   ;      
      
   s       =       t   f   (   '   s   '   )   ;      
      
   %   %       p   i   t   c   h       r   a   t   e       c   o   m   m   a   n   d       s   y   s   t   e   m      
   e   2   q       =       m   i   n   r   e   a   l   (   e   l   e   v   a   t   o   r   _   2   _   o   u   t   s   (   5   )   )   ;      
   %       r   l   t   o   o   l   (   e   2   q   )   ;      
   k   q       =       -   2   0   ;      
      
   e   2   q   _   c   l   o   s   e   d       =       k   q   *   e   2   q   /   (   1   +   k   q   *   e   2   q   )   ;      
   e   2   t       =       m   i   n   r   e   a   l   (   e   2   q   _   c   l   o   s   e   d   /   s   )   ;      
   %       r   l   t   o   o   l   (   e   2   t   )   ;      
   k   t   h   e   t   a       =       1   .   7   5   ;      
   e   2   t   _   c   l   o   s   e       =       k   t   h   e   t   a   *   e   2   t   /   (   1   +   k   t   h   e   t   a   *   e   2   t   )   ;      
      
   %   %       v   e   l   o   c   i   t   y       c   o   m   m   a   n   d       s   y   s   t   e   m      
   t   2   v       =       m   i   n   r   e   a   l   (   t   h   r   u   s   t   _   2   _   o   u   t   s   (   2   )   )   ;      
   %       r   l   t   o   o   l   (   t   2   v   )      
   k   t   h   r   u   s   t       =       2   0   0   0   ;      
      
   %   %   s   i   m       o   u   t   p   u   t   s      
      
   o   u   t   p   u   t       =       s   i   m   (   '   c   o   n   t   r   o   l   l   e   r   '   )   ;      
   x       =       -   o   u   t   p   u   t   .   s   i   m   o   u   t   .   d   a   t   a   (   :   ,   1   )   ;      
   y       =       o   u   t   p   u   t   .   s   i   m   o   u   t   .   d   a   t   a   (   :   ,   2   )   ;      
   v   y       =       o   u   t   p   u   t   .   y   o   u   t   {   6   }   .   V   a   l   u   e   s   .   D   a   t   a   ;      
   v       =       o   u   t   p   u   t   .   y   o   u   t   {   2   }   .   V   a   l   u   e   s   .   D   a   t   a   ;      
      
   g   l   i   d   e   x       =       -   3   8   1   6   2   :   0   ;      
   g   l   i   d   e   y       =       -   g   l   i   d   e   x   *   t   a   n   (   3   *   p   i   /   1   8   0   )   ;      
   r   u   n   w   a   y   x       =       0   :   1   0   :   8   0   0   0   ;      
   r   u   n   w   a   y   y       =       z   e   r   o   s   (   1   ,       l   e   n   g   t   h   (   r   u   n   w   a   y   x   )   )   ;      
      
   %   %       l   o   o   p       t   o       f   i   n   d       t   o   u   c   h       d   o   w   n       s   i   t   e      
   t   o   u   c   h   _   d   o   w   n       =       -   9   9   ;      
   f   o   r       i       =       1   :   (   l   e   n   g   t   h   (   x   )   -   1   )      
                   i   f       y   (   i   )       <       0       &   &       t   o   u   c   h   _   d   o   w   n       =   =       -   9   9      
                                   t   o   u   c   h   _   d   o   w   n       =       x   (   i   )   ;      
                   e   n   d      
   e   n   d      
   v   y   (   e   n   d   )       =       v   y   (   e   n   d   -   1   )   ;      
   d   i   s   p   (   [   n   e   w   l   i   n   e       '   T   o   u   c   h       d   o   w   n       d   i   s   t   a   n   c   e   :       '       n   u   m   2   s   t   r   (   t   o   u   c   h   _   d   o   w   n   )       '       f   t       p   a   s   t       r   u   n   w   a   y       s   t   a   r   t   .       L   a   n   d   i   n   g       s   p   e   e   d       =       '       n   u   m   2   s   t   r   (   v   y   (   e   n   d   )   )       '       f   t   /   s   '   ]   )      
      
      
   h       =       f   i   g   u   r   e   (   )   ;      
   y   y   a   x   i   s       l   e   f   t      
   p   l   o   t   (   g   l   i   d   e   x   ,       g   l   i   d   e   y   ,       '   D   i   s   p   l   a   y   N   a   m   e   '   ,       '   G   l   i   d   e       P   a   t   h   '   ,       '   L   i   n   e   S   t   y   l   e   '   ,       '   -       -   '   ,       '   C   o   l   o   r   '   ,       [   0   .   5       0       0   .   8   ]   )   ;       h   o   l   d       o   n   ;      
   p   l   o   t   (   r   u   n   w   a   y   x   ,       r   u   n   w   a   y   y   ,       '   D   i   s   p   l   a   y   N   a   m   e   '   ,       '   R   u   n   w   a   y   '   ,       '   L   i   n   e   W   i   d   t   h   '   ,       2   ,       '   L   i   n   e   S   t   y   l   e   '   ,       '   -   .   '   ,       '   C   o   l   o   r   '   ,       [   0       0       0   ]   )   ;       h   o   l   d       o   n   ;      
   p   l   o   t   (   x   ,   y   ,       '   D   i   s   p   l   a   y   N   a   m   e   '   ,       '   F   l   i   g   h   t       P   a   t   h   '   ,       '   L   i   n   e   W   i   d   t   h   '   ,       2   ,       '   L   i   n   e   S   t   y   l   e   '   ,       '   -   '   )      
   l   e   g   e   n   d      
      
   x   l   a   b   e   l   (   '   H   o   r   i   z   o   n   t   a   l       L   o   c   a   t   i   o   n       [   f   t   ]   '   )      
   y   l   a   b   e   l   (   '   H   e   i   g   h   t       [   f   t   ]   '   )      
   x   l   i   m   (   [   -   4   2   0   0   0       8   0   0   0   ]   )      
   %       x   l   i   m   (   [   -   2   0   0   0       2   0   0   0   ]   )      
   y   l   i   m   (   [   -   5   0   0       2   5   0   0   ]   )      
      
      
   y   y   a   x   i   s       r   i   g   h   t      
      
   p   l   o   t   (   x   ,       v   y   ,       '   D   i   s   p   l   a   y   N   a   m   e   '   ,       '   V   y   '   ,       '   L   i   n   e   W   i   d   t   h   '   ,       2   )      
   y   l   a   b   e   l   (   '   V   e   r   t   i   c   a   l       v   e   l   o   c   i   t   y       [   f   t   /   s   ]   '   )      
   y   l   i   m   (   [   -   2   5       5   ]   )      
   l   e   g   e   n   d      
   g   r   i   d      
      
   s   e   t   (   h   ,   '   U   n   i   t   s   '   ,   '   I   n   c   h   e   s   '   )   ;       p   o   s       =       g   e   t   (   h   ,   '   P   o   s   i   t   i   o   n   '   )   ;       s   e   t   (   h   ,   '   P   a   p   e   r   P   o   s   i   t   i   o   n   M   o   d   e   '   ,   '   A   u   t   o   '   ,   '   P   a   p   e   r   U   n   i   t   s   '   ,   '   I   n   c   h   e   s   '   ,   '   P   a   p   e   r   S   i   z   e   '   ,   [   p   o   s   (   3   )   ,       p   o   s   (   4   )   ]   )      
   p   r   i   n   t   (   h   ,   '   f   i   g   s   /   f   l   a   r   e   _   o   n   l   y   '   ,   '   -   d   p   d   f   '   ,   '   -   r   0   '   )      
      
   %       h       =       f   i   g   u   r   e   (   )   ;      
   %       p   l   o   t   (   x   ,       v   ,       '   D   i   s   p   l   a   y   N   a   m   e   '   ,       '   V   '   ,       '   L   i   n   e   W   i   d   t   h   '   ,       2   )      
   %       y   l   a   b   e   l   (   '   A   i   r   s   p   e   e   d       [   f   t   /   s   ]   '   )      
   %       l   e   g   e   n   d      
   %       g   r   i   d      
   %          
   %       s   e   t   (   h   ,   '   U   n   i   t   s   '   ,   '   I   n   c   h   e   s   '   )   ;       p   o   s       =       g   e   t   (   h   ,   '   P   o   s   i   t   i   o   n   '   )   ;       s   e   t   (   h   ,   '   P   a   p   e   r   P   o   s   i   t   i   o   n   M   o   d   e   '   ,   '   A   u   t   o   '   ,   '   P   a   p   e   r   U   n   i   t   s   '   ,   '   I   n   c   h   e   s   '   ,   '   P   a   p   e   r   S   i   z   e   '   ,   [   p   o   s   (   3   )   ,       p   o   s   (   4   )   ]   )      
   %       p   r   i   n   t   (   h   ,   '   f   i   g   s   /   v   e   l   o   c   i   t   y   _   l   a   n   d   i   n   g   '   ,   '   -   d   p   d   f   '   ,   '   -   r   0   '   )                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            