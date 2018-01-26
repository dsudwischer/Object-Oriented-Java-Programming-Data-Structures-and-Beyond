package textgen;

import java.util.AbstractList;

import javax.swing.text.AbstractDocument.LeafElement;


/** A class that implements a doubly linked list
 * 
 * @author UC San Diego Intermediate Programming MOOC team
 *
 * @param <E> The type of the elements stored in the list
 */
public class MyLinkedList<E> extends AbstractList<E> {
	LLNode<E> head;
	LLNode<E> tail;
	int size;

	/** Create a new empty LinkedList */
	public MyLinkedList() {
		head = new LLNode<E>(null);
		tail = new LLNode<E>(null);
		size = 0;
		head.next = tail;
		tail.prev = head;
	}

	/**
	 * Appends an element to the end of the list
	 * @param element The element to add
	 */
	public boolean add(E element) // Why is the return type boolean..?
	{
		if(element == null)
		{
			throw new NullPointerException("A NULL element cannot be added to the list.");
		}
		LLNode<E> newNode = new LLNode<E>(element);
		newNode.prev = this.tail.prev;
		newNode.prev.next = newNode;
		newNode.next = this.tail;
		this.tail.prev = newNode;
		this.size += 1;
		return false;
	}

	/** Get the element at position index 
	 * @throws IndexOutOfBoundsException if the index is out of bounds. */
	public E get(int index) 
	{
		if(!isValidIndex(index))
		{
			throw new IndexOutOfBoundsException("The requested index is beyond the bounds.");
		}
		LLNode<E> curNode = getNode(index);
		return curNode.data;
	}

	/**
	 * Add an element to the list at the specified index
	 * @param The index where the element should be added
	 * @param element The element to add
	 */
	public void add(int index, E element) 
	{
		if(isNull(element))
		{
			throw new NullPointerException("NULL elements cannot be inserted.");
		}
		LLNode<E> curNode = getNode(index);
		// We insert the new Node straight before curNode
		LLNode<E> newNode = new LLNode<E>(element);
		newNode.prev = curNode.prev;
		newNode.next = curNode;
		curNode.prev.next = newNode;
		curNode.prev = newNode;
		this.size += 1;
	}


	/** Return the size of the list */
	public int size() 
	{
		return this.size;
	}

	/** Remove a node at the specified index and return its data element.
	 * @param index The index of the element to remove
	 * @return The data element removed
	 * @throws IndexOutOfBoundsException If index is outside the bounds of the list
	 * 
	 */
	public E remove(int index) 
	{
		if(!isValidIndex(index))
		{
			throw new IndexOutOfBoundsException();
		}
		LLNode<E> curNode = getNode(index);
		curNode.prev.next = curNode.next;
		curNode.next.prev = curNode.prev;
		this.size -= 1;
		return curNode.data;
	}

	/**
	 * Set an index position in the list to a new element
	 * @param index The index of the element to change
	 * @param element The new element
	 * @return The element that was replaced
	 * @throws IndexOutOfBoundsException if the index is out of bounds.
	 */
	public E set(int index, E element) 
	{
		if(!isValidIndex(index)) { throw new IndexOutOfBoundsException(); }
		int curIndex = 0;
		LLNode<E> curNode = this.head.next;
		while(curIndex < index)
		{
			curNode = curNode.next;
			curIndex++;
		}
		E oldData = curNode.data;
		curNode.data = element;
		return oldData;
	} 
	
	/** Returns the node at the specified index */
	private LLNode<E> getNode(int index)
	{
		if(!isValidIndex(index)) { throw new IndexOutOfBoundsException(); }
		int curIndex = 0;
		LLNode<E> curNode = this.head.next; // Since 0 <= index < this.size, we have at least
		// one non-dummy node.
		while(curIndex < index)
		{
			curNode = curNode.next;
			curIndex++;
		}
		return curNode;
	}
	
	/** Checks if an element is null */
	private boolean isNull(E element)
	{
		return element == null;
	}
	
	/** Checks if the specified index is within bounds */
	private boolean isValidIndex(int index)
	{
		return index >= 0 && index < this.size();
	}
}

class LLNode<E> 
{
	LLNode<E> prev;
	LLNode<E> next;
	E data;

	public LLNode(E e) 
	{
		this.data = e;
		this.prev = null;
		this.next = null;
	}
	
	public LLNode(E e, LLNode<E> prevNode, LLNode<E> nextNode)
	{
		this.data = e;
		this.next = nextNode;
		this.prev = prevNode;
		prevNode.next = this;
		nextNode.prev = this;
	}

}
