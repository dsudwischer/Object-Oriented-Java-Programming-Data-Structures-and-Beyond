/**
 * 
 */
package textgen;

import static org.junit.Assert.*;

import java.util.LinkedList;

import org.junit.Before;
import org.junit.Test;

/**
 * @author UC San Diego MOOC team
 *
 */
public class MyLinkedListTester {

	private static final int LONG_LIST_LENGTH =10; 

	MyLinkedList<String> shortList;
	MyLinkedList<Integer> emptyList;
	MyLinkedList<Integer> longerList;
	MyLinkedList<Integer> list1;
	
	/**
	 * @throws java.lang.Exception
	 */
	@Before
	public void setUp() throws Exception {
		// Feel free to use these lists, or add your own
	    shortList = new MyLinkedList<String>();
		shortList.add("A");
		shortList.add("B");
		emptyList = new MyLinkedList<Integer>();
		longerList = new MyLinkedList<Integer>();
		for (int i = 0; i < LONG_LIST_LENGTH; i++)
		{
			longerList.add(i);
		}
		list1 = new MyLinkedList<Integer>();
		list1.add(65);
		list1.add(21);
		list1.add(42);
		
	}

	
	/** Test if the get method is working correctly.
	 */
	/*You should not need to add much to this method.
	 * We provide it as an example of a thorough test. */
	@Test
	public void testGet()
	{
		//test empty list, get should throw an exception
		try {
			emptyList.get(0);
			fail("Check out of bounds");
		}
		catch (IndexOutOfBoundsException e) {
			
		}
		
		// test short list, first contents, then out of bounds
		assertEquals("Check first", "A", shortList.get(0));
		assertEquals("Check second", "B", shortList.get(1));
		
		try {
			shortList.get(-1);
			fail("Check out of bounds");
		}
		catch (IndexOutOfBoundsException e) {
		
		}
		try {
			shortList.get(2);
			fail("Check out of bounds");
		}
		catch (IndexOutOfBoundsException e) {
		
		}
		// test longer list contents
		for(int i = 0; i<LONG_LIST_LENGTH; i++ ) {
			assertEquals("Check "+i+ " element", (Integer)i, longerList.get(i));
		}
		
		// test off the end of the longer array
		try {
			longerList.get(-1);
			fail("Check out of bounds");
		}
		catch (IndexOutOfBoundsException e) {
		
		}
		try {
			longerList.get(LONG_LIST_LENGTH);
			fail("Check out of bounds");
		}
		catch (IndexOutOfBoundsException e) {
		}	
	}
	
	
	/** Test removing an element from the list.
	 * We've included the example from the concept challenge.
	 * You will want to add more tests.  */
	@Test
	public void testRemove()
	{
		int a = list1.remove(0);
		assertEquals("Remove: check a is correct ", 65, a);
		assertEquals("Remove: check element 0 is correct ", (Integer)21, list1.get(0));
		assertEquals("Remove: check size is correct ", 2, list1.size());
		
		// Add more tests here
		
		try
		{
			list1.remove(-1);
			fail("Cannot remove at index < 0.");
		}
		catch(IndexOutOfBoundsException e) { }
		try
		{
			longerList.remove(LONG_LIST_LENGTH);
			fail("Cannot remove at index >= size");
		}
		catch(IndexOutOfBoundsException e) { }
		
		int b = longerList.remove(5);
		assertEquals("Remove: check middle ", 5, b);
		assertEquals("Remove: check prior ", (Integer) 6, longerList.get(5));
		
		try
		{
			int c = emptyList.remove(0);
			fail("Cannot remove from an empty list");
		}
		catch(Exception e) {	
		}
		
		
	}
	
	/** Test adding an element into the end of the list, specifically
	 *  public boolean add(E element)
	 * */
	@Test
	public void testAddEnd()
	{
        // implement this test
		try
		{
			emptyList.add(null);
			fail("AddEnd: Cannot add NULL");
		}
		catch(NullPointerException e) {}
		try
		{	
			emptyList.add(1, 2);
			fail("AddEnd: Cannot add at indices > size");
		}
		catch(IndexOutOfBoundsException e) {}
		try
		{	
			emptyList.add(-1, 2);
			fail("AddEnd: Cannot add at indices < 0");
		}
		catch(IndexOutOfBoundsException e) {}
		emptyList.add(1);
		assertEquals("AddEnd: add to empty list ", (Integer) 1, emptyList.get(0));
		assertEquals("AddEnd: check size ", 1, emptyList.size());
	}

	
	/** Test the size of the list */
	@Test
	public void testSize()
	{
		// implement this test
		assertEquals("Size: check empty list", 0, emptyList.size());
		assertEquals("Size: check longer List", LONG_LIST_LENGTH, longerList.size());
	}

	
	
	/** Test adding an element into the list at a specified index,
	 * specifically:
	 * public void add(int index, E element)
	 * */
	@Test
	public void testAddAtIndex()
	{
        // implement this test
		try
		{
			emptyList.add(null);
			fail("AddAtIndex: Cannot add NULL");
		}
		catch(NullPointerException e) {}
		emptyList.add(0, 1);
		assertEquals("AddAtIndex: check adding empty ", (Integer) 1, emptyList.get(0));
		assertEquals("AddAtIndex: check size ", 1, emptyList.size());
		longerList.add(LONG_LIST_LENGTH, LONG_LIST_LENGTH);
		assertEquals("AddAtIndex: check adding back ", LONG_LIST_LENGTH + 1, longerList.size());
		list1.add(1, 10);
		assertEquals("AddAtIndex: check adding middle (new insert)", (Integer) 10, list1.get(1));
		assertEquals("AddAtIndex: check adding middle (next)", (Integer) 65, list1.get(0));
		assertEquals("AddAtIndex: check adding middle (prev)", (Integer) 21, list1.get(2));
		assertEquals("AddAtIndex: check adding middle (size) ", 4, list1.size());
	}
	
	/** Test setting an element in the list */
	@Test
	public void testSet()
	{
	    // implement this test
		try
	    {
	    	emptyList.set(0, 1);
	    	fail("Set: check bounds.");
	    }
	    catch(IndexOutOfBoundsException e) {}
		try
	    {
	    	list1.set(-1, 1);
	    	fail("Set: check lower bounds.");
	    }
	    catch(IndexOutOfBoundsException e) {}
		try
	    {
	    	list1.set(3, 1);
	    	fail("Set: check upper bounds.");
	    }
	    catch(IndexOutOfBoundsException e) {}
	    try
	    {
	    	longerList.set(0, null);
	    	fail("Set: check null value");
	    }
	    catch(NullPointerException e) {}
	    list1.set(2, 3);
	    assertEquals("Set: check setting value ", (Integer) 3, list1.get(2));
	}
	
	
	// Optionally add more test methods.
	
}
