{-# LANGUAGE KindSignatures #-}
-- | Test Module.
module Category (Functor') where

import Data.List
import Control.Arrow
import Control.Monad
import Control.Applicative
import Graphics.Matplotlib
import Data.MarkovChain
import System.Random (mkStdGen)


data Either' a b = Left a | Right b


class Functor' (f :: * -> *) where
  fmap' :: (a -> b) -> f a -> f b

class (Functor' f) => MyFunctor' f where
  (<$$) :: a -> f b -> f a

instance Functor' [] where
  fmap' = map

instance MyFunctor' [] where
  a <$$ [] = []
  a <$$ [b] = [a]


data L a = N | L a

fib :: Int -> Int
fib n
  | n == 0 = 0
  | n == 1 = 1
  | n >  1 = fib (n - 2) + fib (n - 1)
