use core::marker::PhantomData;

use crate::new_safe_metrics::uom_helpers::{HasBaseUnitSystem, HasDimension};

trait ConsumableMetricRepresentation {}

trait MetricConsumer {}

pub trait ConsumableMetric<Consumer, Value, Unit, Rep> 
    where 
        Consumer: MetricConsumer,
        Value: HasBaseUnitSystem,
        Unit: HasDimension, //Todo Hans: At this point, unit is not bounded to be of the correct type, e.g. it could be "hour" for an angle. However, compile errors will be thrown later
        Rep: ConsumableMetricRepresentation //Can we avoid this?
{
    type Conversion: Lens<uom::si::Quantity::<Unit::Dimension, Value::BaseUnits, Value>, Rep>;
    // type Conversion : Lens<uom::si::Quantity::<Unit::Dimension, Value::BaseUnits, Value>, impl ConsumableMetricRepresentation>;

    // fn from_consumer(representation: impl ConsumableMetricRepresentation) -> uom::si::Quantity::<Unit::Dimension, Value::BaseUnits, Value> {
    //     todo!();
    // }

    // fn to_consumer(quantity: &uom::si::Quantity::<Unit::Dimension, Value::BaseUnits, Value>) -> impl ConsumableMetricRepresentation {
    //     todo!();
    // }

    /*
    type Lens: Lens<Quantity<...>, ConsumableMetricRepre>
    
     */
}

pub trait ConstLens : Sized {
    
}

macro_rules! impl_mytrait_for_all_const_types {
    ($struct:ident, $trait:ident) => {
        macro_rules! __impl {
            ($($t:ty),*) => {
                $(
                    impl<const N: $t> $trait for $struct<N> {
                        fn describe() {
                            println!("Implemented for {} with const {}", stringify!($t), N);
                        }
                    }
                )*
            };
        }

        __impl!(
            u8, u16, u32, u64, u128, usize,
            i8, i16, i32, i64, i128, isize,
            bool, char
        );
    };
}


struct MyStruct<const N: usize>;

trait MyTrait {
    fn describe();
}

impl_mytrait_for_all_const_types!(MyStruct, MyTrait);

// impl<T> ConstLens<T, T> for Identity<T> {
//     type Inverse = Identity<T>;
//     fn eval(self, from: T) -> T {
//        return from;
//     }
//     fn invert(self) -> Self::Inverse {
//         return self;
//     }
// }


pub trait Lens<From, To> : Sized {
    type Inverse: Lens<To, From>;
    fn eval(self, from: From) -> To;
    fn invert(self) -> Self::Inverse;

    fn new<T>(value: T) -> impl Lens<T, T> {
        return Identity{ value };
    }

    fn add<With, NewTo>(self, with: With) -> impl Lens<From, NewTo>
        where 
            To: core::ops::Add<With, Output = NewTo>,
            NewTo: core::ops::Sub<With, Output = To>,
    {
        return CompositeLens{l1: self, l2: Add { from: PhantomData, with }, from: PhantomData, via: PhantomData, to: PhantomData};
    }

    fn sub<With, NewTo>(self, with: With) -> impl Lens<From, NewTo>
        where 
            To: core::ops::Sub<With, Output = NewTo>,
            NewTo: core::ops::Add<With, Output = To>,
    {
        return CompositeLens{l1: self, l2: Sub { from: PhantomData, with }, from: PhantomData, via: PhantomData, to: PhantomData};
    }
}
impl<L1: Lens<From, Via>, L2: Lens<Via, To>, From, Via, To> Lens<From, To> for CompositeLens<L1, L2, From, Via, To>{
    type Inverse = CompositeLens<L2::Inverse, L1::Inverse, To, Via, From>;
    fn eval(self, from: From) -> To {
        return self.l2.eval(self.l1.eval(from));
    }
    fn invert(self) -> Self::Inverse {
        return CompositeLens{ l1: self.l2.invert(), l2: self.l1.invert(), from: PhantomData, via: PhantomData, to: PhantomData};
    }
}
impl<T> Lens<T, T> for Identity<T> {
    type Inverse = Identity<T>;
    fn eval(self, from: T) -> T {
       return from;
    }
    fn invert(self) -> Self::Inverse {
        return self;
    }
}
impl<From, With, To> Lens<From, To> for Add<From, With>
    where 
        From: core::ops::Add<With, Output = To>,
        To: core::ops::Sub<With, Output = From>,
{
    type Inverse = Sub<To, With>;
    fn eval(self, from: From) -> To {
        return from + self.with;
    }
    fn invert(self) -> Self::Inverse {
        return Sub{with: self.with, from: PhantomData};
    }
}
impl<From, With, To> Lens<From, To> for Sub<From, With>
    where 
        From: core::ops::Sub<With, Output = To>,
        To: core::ops::Add<With, Output = From>,
{
    type Inverse = Add<To, With>;
    fn eval(self, from: From) -> To {
        return from - self.with;
    }
    fn invert(self) -> Self::Inverse {
        return Add{with: self.with, from: PhantomData};
    }
}

struct CompositeLens<L1: Lens<From, Via>, L2: Lens<Via, To>, From, Via, To> {
    l1: L1,
    l2: L2,
    from: PhantomData<From>,
    via: PhantomData<Via>,
    to: PhantomData<To>,
}

struct Identity<T> {
    value: T,
} 

struct Add<From, With>
{
    from: PhantomData<From>,
    with: With,
}

struct Sub<From, With>{
    from: PhantomData<From>,
    with: With,
}