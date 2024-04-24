//! Assorted reusable utility thingies.

use core::{sync::atomic::{AtomicBool, Ordering}, cell::UnsafeCell};

use lilos::atomic::AtomicExt;

/// Stores a `T` at static scope such that one exclusive reference (`&mut`) can
/// be produced.
pub struct StaticResource<T> {
    taken: AtomicBool,
    inner: UnsafeCell<T>,
}

impl<T> StaticResource<T> {
    pub const fn new(initializer: T) -> Self {
        Self {
            taken: AtomicBool::new(false),
            inner: UnsafeCell::new(initializer),
        }
    }

    /// Gets a reference to the resource. Only works once, to avoid aliasing.
    /// Any subsequent calls will panic.
    ///
    /// This requires `self` to be `'static` to ensure that you can't use this
    /// to generate allegedly-`'static` references pointing into, say, the
    /// stack.
    #[allow(clippy::mut_from_ref)] // don't worry, it only succeeds once
    #[track_caller]
    #[inline(always)]
    pub fn take(&'static self) -> &'static mut T {
        // Detect the _second_ attempt to pass this point.
        let already_taken = self.taken.swap_polyfill(true, Ordering::SeqCst);
        if already_taken {
            // Currently allowing this panic to generate a new panic site for
            // every caller, because getting track_caller seems like a big win.
            // If this proves to use excessive space, this could be out-lined
            // into an inline(never)/cold fn to effectively monomorphize it, at
            // the cost of knowing _which_ take failed without a backtrace.
            panic!();
        }

        // Safety: we have ensured that execution will only reach this point
        // once in the lifetime of the storage. Thus we can produce a &mut
        // without risk of aliasing, which is otherwise the main risk of taking
        // a &mut out of an UnsafeCell.
        unsafe { &mut *self.inner.get() }
    }
}

/// Safety: we protect our inner `T` so we can be `Sync` (and thus stored at
/// static scope, potentially reached by multiple threads) without `T` itself
/// being `Sync`.
unsafe impl<T> Sync for StaticResource<T> {}
