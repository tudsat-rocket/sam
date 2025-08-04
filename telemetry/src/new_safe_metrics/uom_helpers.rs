///This trait can be used to automatically get the BaseUnitSystem for a given type
pub trait HasBaseUnitSystem: uom::num_traits::Num + uom::Conversion<Self>{
    type BaseUnits: uom::si::Units<Self> + ?Sized;
}

///This trait can be used to automatically get the Dimension for a given unit
pub trait HasDimension : uom::si::Unit {
    type Dimension : uom::si::Dimension + ?Sized;
}

//---------------------------------------------------//
//------------impl HasBaseUnitSystem-----------------//
//---------------------------------------------------//

impl HasBaseUnitSystem for f32 {
    type BaseUnits = uom::si::SI<f32>;
}

//---------------------------------------------------//
//---------------impl HasDimension-------------------//
//---------------------------------------------------//

impl<U : uom::si::length::Unit> HasDimension for U  {
    type Dimension = uom::si::length::Dimension;
}